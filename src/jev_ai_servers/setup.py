from setuptools import find_packages, setup

package_name = 'jev_ai_servers'

# Built-in backends. Third-party packages register more under the same groups.
JEV_BACKENDS = [
    'mock = jev_ai_servers.jev_backends.mock:MockJev',
    'replay = jev_ai_servers.jev_backends.replay:ReplayJev',
    'typesafe = jev_ai_servers.jev_backends.typesafe_backend:TypeSafeJev',
    'llama_ros = jev_ai_servers.jev_backends.llama_ros:LlamaRosJev',
]
LLM_BACKENDS = [
    'mock = jev_ai_servers.llm_backends.mock:MockLlm',
    'replay = jev_ai_servers.llm_backends.replay:ReplayLlm',
    'anthropic = jev_ai_servers.llm_backends.anthropic_backend:AnthropicLlm',
    'openai_compat = jev_ai_servers.llm_backends.openai_compat:OpenAICompatLlm',
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Lobell',
    maintainer_email='jasonlobell123@gmail.com',
    description='jev_ai servers: jev_server, llm_server and built-in backends',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jev_server = jev_ai_servers.jev_server:main',
            'llm_server = jev_ai_servers.llm_server:main',
            'decide_server = jev_ai_servers.decide_server:main',
            'decision_logger = jev_ai_servers.decision_logger:main',
            'dashboard = jev_ai_servers.dashboard:main',
        ],
        'jev_ai.jev_backends': JEV_BACKENDS,
        'jev_ai.llm_backends': LLM_BACKENDS,
    },
)
