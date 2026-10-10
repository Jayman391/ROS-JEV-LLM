"""Backend plugin interfaces for jev_server and llm_server (pure Python, no rclpy).

A backend is a class registered under an entry-point group (setup.py):

    jev_ai.jev_backends:  <name> = <module>:<Class>     (subclass of JevBackend)
    jev_ai.llm_backends:  <name> = <module>:<Class>     (subclass of LlmBackend)

The server constructs it with the parameters under its name (e.g. ``mock.seed``) and the API key
read from ``key_env`` or ``api_key_file``, awaits ``check()`` at configure time, then awaits
``decide()`` / ``reason()`` on a dedicated asyncio thread. Cancelling that task must abort the
request (e.g. close the HTTP stream).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from importlib import import_module
from importlib.metadata import entry_points
from typing import Any, Callable


class BackendError(RuntimeError):
    """A call or configuration failure. ``retryable``: 429/5xx/network, worth one more try."""

    def __init__(self, msg: str, retryable: bool = False):
        super().__init__(msg)
        self.retryable = retryable


class _Backend:
    name = ''
    key_env = ''             # env var holding the API key; '' = no key needed

    def __init__(self, params: dict[str, Any], api_key: str = ''):
        self.params = params
        self.api_key = api_key
        self.node = None     # set by the server (for ROS-native backends such as llama_ros)

    async def check(self) -> None:
        """Configure-time validation (key, endpoint, model). Raise BackendError on failure."""

    async def close(self) -> None:
        pass


# --- Jev ---------------------------------------------------------------------------------------

@dataclass
class JevQuestion:
    name: str
    type: int                 # jev_ai_msgs/Question NOUL=0 | CHOICE=1 | SCORE=2
    instructions: str
    classes: list[str]
    criteria: list[str]       # aligned with classes; '' = none


@dataclass
class JevAnswer:
    probabilities: list[float]          # aligned with the question's classes
    source_confidence: float = math.nan  # what the model itself reported


@dataclass
class JevResult:
    model: str
    answers: list[JevAnswer | None]     # aligned with questions; None = that question failed
    question_errors: list[str]          # aligned with questions; '' = ok
    input_tokens: int = 0


class JevBackend(_Backend):
    async def decide(self, state: list[tuple[str, str]], questions: list[JevQuestion],
                     model: str) -> JevResult:
        raise NotImplementedError


# --- LLM ---------------------------------------------------------------------------------------

@dataclass
class ReasonRequest:
    system: str
    context: str
    response_schema: str = ''           # JSON Schema text; '' = free text
    model: str = ''                     # '' = backend default
    options: dict[str, str] = field(default_factory=dict)   # e.g. effort=low, max_tokens=800
    stream_text: bool = False


@dataclass
class ReasonResult:
    text: str
    json: str = ''                      # set when response_schema was given and output is JSON
    model: str = ''
    input_tokens: int = 0
    output_tokens: int = 0
    cost_usd: float = math.nan          # NaN = the server prices it from its pricing table


Feedback = Callable[[str, str], None]   # (stage, delta): queued|started|first_token|streaming|done


class LlmBackend(_Backend):
    async def reason(self, req: ReasonRequest, feedback: Feedback) -> ReasonResult:
        raise NotImplementedError


# --- loading -----------------------------------------------------------------------------------

def load_backend(group: str, name: str, builtin: dict[str, str]) -> type:
    """Installed entry points first (third-party plugins), then the server's built-in table, so
    the servers also run from a source tree."""
    for ep in entry_points(group=group):
        if ep.name == name:
            return ep.load()
    if name not in builtin:
        known = sorted({ep.name for ep in entry_points(group=group)} | set(builtin))
        raise BackendError(f'unknown backend {name!r} in {group}; known: {", ".join(known)}')
    module, cls = builtin[name].split(':')
    return getattr(import_module(module), cls)
