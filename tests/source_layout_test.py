"""Guard module ownership: controller math is private; retired paths must not reappear."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
controller = ROOT / 'src/motion_controllers'
retired = ('src/numerics', 'src/input', 'src/web_gui', 'src/simulation_ipc',
           'apps/input', 'apps/web_gui', 'web_gui')
for old in retired:
    assert not (ROOT / old).exists(), f'retired module path exists: {old}'
for base in ('src', 'apps', 'visualization'):
    for path in (ROOT / base).rglob('*'):
        if path.suffix not in ('.cpp', '.hpp', '.cc', '.h') or 'node_modules' in path.parts:
            continue
        text = path.read_text()
        if not path.is_relative_to(controller):
            assert 'motion_controllers/numerics/' not in text, f'external math dependency: {path}'
            assert not re.search(r'\bnumeric::', text), f'external controller numerics use: {path}'
        assert not re.search(r'#include\s+"(?:numerics|input|web_gui|simulation_ipc)/', text), path
backend = (ROOT / 'src/robot_model/backend.hpp').read_text()
assert '#include <array>' in backend
cmake = (ROOT / 'CMakeLists.txt').read_text()
assert 'add_library(numerics ' not in cmake
assert 'target_link_libraries(robot_model PUBLIC qhrr05_common yaml-cpp)' in cmake
assert (ROOT / 'apps/interfaces/web_gui/frontend/server.mjs').is_file()
assert (ROOT / 'src/interfaces/CMakeLists.txt').is_file()
print('PASS: controller-private numerics, independent model backend, interfaces grouping, IPC layout')

for path in controller.glob('*.cpp'):
    text = path.read_text()
    assert 'SimulationData' not in text and 'ObservationOrigin' not in text, path
control_text = (ROOT / 'src/control/control.cpp').read_text()
assert 'simulation_ipc' not in control_text and 'position_gt' not in control_text
print('PASS: motion/control consumes common observations without simulation IPC or GT')
