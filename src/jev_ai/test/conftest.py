import copy

import pytest

from jev_ai.core.catalog import load_catalog

# The FRAMEWORK §4 example catalog.
PATROL = {
    'catalog_version': 1,
    'name': 'patrol',
    'question_sets': {
        'next_step': {
            'state_keys': ['task', 'energy', 'link', 'hazards', 'detection'],
            'tick_hz': 1.0,
            'augment': True,
            'group': 'fleet',
            'questions': {
                'next_action': {
                    'type': 'choice',
                    'instructions': "Choose the robot's next action.",
                    'act': 'sample',
                    'classes': [
                        {'name': 'continue', 'criteria': 'Keep doing the assigned task.',
                         'reversible': True},
                        {'name': 'inspect', 'criteria': 'Look closer at the detection.',
                         'reversible': 'until_commit'},
                        {'name': 'halt_now', 'criteria': 'Emergency stop; critical failure only.',
                         'reversible': False},
                    ],
                    'fallback': 'continue',
                    'tau': 0.5,
                    'kappa': 0.7,
                    'coupling': 0.5,
                    'argmax_margin': 0.1,
                },
                'urgency': {
                    'type': 'score',
                    'instructions': 'How urgent is resolving the detection?',
                    'classes': [{'name': 'none'}, {'name': 'low'}, {'name': 'high'}],
                    'act': 'expectation',
                },
                'report_now': {
                    'type': 'noul',
                    'instructions': 'Should the robot report now?',
                    'criteria': {'true': 'Something changed worth reporting.',
                                 'false': 'Nothing new.'},
                    'act': 'threshold',
                    'threshold': 0.5,
                },
            },
            'llm_params': {'inspect_altitude_m': {'min': 8, 'max': 15, 'default': 10}},
            'llm': {'system': 'You are the onboard reasoning module of {agent}.'},
        },
    },
    'policy': {},
}

STATE = {'task': 'patrolling', 'energy': 'ok (70%)', 'link': 'good', 'hazards': 'none',
         'detection': 'none'}


@pytest.fixture
def raw():
    return copy.deepcopy(PATROL)


@pytest.fixture
def catalog(raw):
    return load_catalog(raw)


@pytest.fixture
def qset(catalog):
    return catalog.question_sets['next_step']
