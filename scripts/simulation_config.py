#!/usr/bin/env python3
"""Derive a session manifest from the existing deployment configuration.

No control/simulation policy is copied here. Only the CAN transport and the
presence of the existing simulator process differ from the hardware manifest.
"""
import argparse
from copy import deepcopy
from pathlib import Path
import yaml


def write_hal(root, output, hardware=False):
    source = root / 'config/can_hal/can_hal.yaml'
    settings = yaml.safe_load(source.read_text())
    settings['interface'] = 'vcan0'
    settings['motor_config'] = str((source.parent / settings['motor_config']).resolve())
    if not hardware:
        settings['poll_released'] = True
    output.write_text(yaml.safe_dump(settings, sort_keys=False))
    return output


def prepare(root, output, *, hardware=False, headless=False, without_can=False,
            template=None, shm_name=None):
    root, output = Path(root).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = Path(template).resolve() if template else root / 'config/daemon/daemon.yaml'
    manifest = yaml.safe_load(source.read_text())
    if template and manifest.get('runtime_mode') != 'simulation':
        raise ValueError('explicit simulation manifest requires runtime_mode: simulation')
    manifest['runtime_mode'] = 'hardware' if hardware else 'simulation'
    if shm_name is not None:
        manifest['shm_name'] = shm_name
    if manifest.get('log_directory'):
        manifest['log_directory'] = str((source.parent / manifest['log_directory']).resolve())
    if not template and not hardware and 'terminal_layout' in manifest:
        manifest['terminal_layout']['columns'] = 3
    hal = write_hal(root, output / 'can.yaml', hardware)
    sections = ('processes', 'debug_processes') if template else ('processes',)
    for section in sections:
        if section not in manifest:
            continue
        for process in manifest[section]:
            work = (source.parent / process.get('working_directory', '.')).resolve()
            process['working_directory'] = str(work)
            args = process['args']
            for i, arg in enumerate(args):
                if arg == '{simulation_can_config}' or (not arg.startswith('--') and arg != '{shm_name}' and
                        (work / arg).resolve() == root / 'config/can_hal/can_hal.yaml'):
                    args[i] = str(hal)
    if not template:
        if hardware:
            control = yaml.safe_load((root / 'config/control/control.yaml').read_text())
            model = (root / 'config/control' / control['model_path']).resolve()
        else:
            model = root / 'models/qhrr-05/scene.xml'
        simulation = dict(name='simulation', exec_path='simulation', working_directory=str(root),
                          wait_ready=True, ready_timeout_ms=20000,
                          args=['--model', str(model), '--control-config', str(root / 'config/control/control.yaml'),
                                '--shm-name', '{shm_name}', '--hal-config', str(hal),
                                '--simulation-config', str(root / 'config/simulation/simulation.yaml')])
        if headless:
            simulation['args'].append('--headless')
        index = next((i + 1 for i, p in enumerate(manifest['processes']) if p['name'] == 'can_hal'), 0)
        manifest['processes'].insert(index, simulation)
        if without_can:
            debug = deepcopy([p for p in manifest['processes'] if p['name'] != 'can_hal'])
            for process in debug:
                if process['name'] == 'simulation':
                    args = process['args']
                    for option in ('--hal-config', '--simulation-config'):
                        i = args.index(option)
                        del args[i:i + 2]
                    args.append('--without-can')
                elif process['name'] == 'control':
                    process['args'].append('--without-can')
            manifest['debug_processes'] = debug
    target = output / 'daemon.yaml'
    target.write_text(yaml.safe_dump(manifest, sort_keys=False))
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--without-can', action='store_true')
    parser.add_argument('--template', type=Path)
    parser.add_argument('--shm-name')
    parser.add_argument('--hal-only', action='store_true')
    args = parser.parse_args()
    if args.hal_only:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        write_hal(args.root.resolve(), args.output, args.hardware)
    else:
        prepare(args.root, args.output, hardware=args.hardware, headless=args.headless,
                without_can=args.without_can, template=args.template, shm_name=args.shm_name)
