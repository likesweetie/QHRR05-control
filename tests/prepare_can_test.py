"""Check hardware CAN preparation without changing host network interfaces."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_can', ROOT / 'scripts/prepare_can.py')
prepare_can = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_can)
REAL_ADMIN_IP = prepare_can.admin_ip


class PrepareCanTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='qhrr-prepare-can-')
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        (root / 'config').mkdir()
        self.manifest = root / 'config/daemon.json'
        self.hal = root / 'can.json'
        self.manifest.write_text(json.dumps({
            'runtime_mode': 'hardware',
            'processes': [{
                'name': 'can_hal', 'working_directory': '..', 'args': ['can.json', '/test'],
            }],
        }))
        self.settings = {'interface': 'test_can7', 'bitrate': 1000000}
        self.link = {
            'ifname': 'test_can7', 'ifindex': 42, 'flags': [],
            'linkinfo': {'info_kind': 'can', 'info_data': {}},
        }
        self.operations = []
        self.fail_operation = None
        self.ignore_mutations = False
        admin_patch = patch.object(prepare_can, 'admin_ip', return_value=['sudo', '--', 'ip'])
        self.admin = admin_patch.start()
        self.addCleanup(admin_patch.stop)
        run_patch = patch.object(prepare_can, 'run', side_effect=self.run_ip)
        run_patch.start()
        self.addCleanup(run_patch.stop)

    def run_ip(self, command):
        if command == ['ip', '-json', '-details', 'link', 'show', 'dev', 'test_can7']:
            return json.dumps([self.link])
        self.assertEqual(command[:7], ['sudo', '--', 'ip', 'link', 'set', 'dev', 'test_can7'])
        operation = command[7:]
        self.operations.append(operation)
        if operation == self.fail_operation:
            raise RuntimeError('test setup failure')
        if not self.ignore_mutations:
            if operation == ['up']:
                self.link['flags'] = ['UP']
            elif operation == ['down']:
                self.link['flags'] = []
            else:
                self.assertEqual(operation[:3], ['type', 'can', 'bitrate'])
                self.link['linkinfo']['info_data']['bittiming'] = {'bitrate': int(operation[3])}
        return ''

    def prepare(self):
        self.hal.write_text(json.dumps(self.settings))
        prepare_can.prepare(self.manifest)

    def set_bitrate(self, value):
        self.link['linkinfo']['info_data']['bittiming'] = {'bitrate': value}

    def test_ready_interface_does_not_require_privileges(self):
        self.link['flags'] = ['UP']
        self.set_bitrate(1000000)
        self.prepare()
        self.admin.assert_not_called()
        self.assertEqual(self.operations, [])

    def test_down_matching_bitrate_only_brings_up(self):
        self.set_bitrate(1000000)
        self.prepare()
        self.assertEqual(self.operations, [['up']])

    def test_unconfigured_interface_gets_bitrate_and_up(self):
        self.prepare()
        self.assertEqual(self.operations, [['type', 'can', 'bitrate', '1000000'], ['up']])
        self.assertEqual(self.link['ifindex'], 42)
        self.assertIn('UP', self.link['flags'])
        self.assertEqual(prepare_can.bitrate(self.link), 1000000)

    def test_up_wrong_bitrate_is_taken_down_before_configuring(self):
        self.link['flags'] = ['UP']
        self.set_bitrate(500000)
        self.prepare()
        self.assertEqual(self.operations, [['down'], ['type', 'can', 'bitrate', '1000000'], ['up']])

    def test_explicit_vcan_does_not_get_physical_bitrate(self):
        self.link['linkinfo']['info_kind'] = 'vcan'
        self.settings.pop('bitrate')
        self.prepare()
        self.assertEqual(self.operations, [['up']])

    def test_wrong_kind_is_not_modified(self):
        self.link['linkinfo']['info_kind'] = 'dummy'
        with self.assertRaisesRegex(ValueError, 'not CAN/vcan'):
            self.prepare()
        self.admin.assert_not_called()

    def test_missing_device_is_not_created(self):
        with patch.object(prepare_can, 'read_link', side_effect=RuntimeError('device not found')):
            with self.assertRaisesRegex(RuntimeError, 'device not found'):
                self.prepare()
        self.assertEqual(self.operations, [])

    def test_invalid_bitrate_is_rejected_before_any_mutation(self):
        for value in [None, True, '1000000', 0, -1, 1000001]:
            with self.subTest(value=value):
                self.settings['bitrate'] = value
                with self.assertRaisesRegex(ValueError, 'bitrate must be an integer'):
                    self.prepare()
        self.admin.assert_not_called()

    def test_failed_configuration_does_not_attempt_up(self):
        self.fail_operation = ['type', 'can', 'bitrate', '1000000']
        with self.assertRaisesRegex(RuntimeError, 'test setup failure'):
            self.prepare()
        self.assertEqual(self.operations, [self.fail_operation])

    def test_configuration_is_verified_after_commands(self):
        self.ignore_mutations = True
        with self.assertRaisesRegex(RuntimeError, 'did not become UP'):
            self.prepare()

    def test_simulation_manifest_is_rejected(self):
        self.manifest.write_text(json.dumps({'runtime_mode': 'simulation'}))
        with self.assertRaisesRegex(ValueError, 'runtime_mode: hardware'):
            self.prepare()
        self.admin.assert_not_called()

    def test_cap_net_admin_avoids_sudo(self):
        with patch.object(Path, 'read_text', return_value='CapEff:\t0000000000001000\n'):
            self.assertEqual(REAL_ADMIN_IP(), ['ip'])

    def test_without_capability_only_ip_is_elevated(self):
        with patch.object(Path, 'read_text', return_value='CapEff:\t0000000000000000\n'), \
             patch.object(prepare_can.shutil, 'which', return_value='/usr/bin/sudo'):
            self.assertEqual(REAL_ADMIN_IP(), ['sudo', '--', 'ip'])

    def test_missing_sudo_reports_privilege_requirement(self):
        with patch.object(Path, 'read_text', return_value='CapEff:\t0000000000000000\n'), \
             patch.object(prepare_can.shutil, 'which', return_value=None):
            with self.assertRaisesRegex(RuntimeError, 'CAP_NET_ADMIN or sudo'):
                REAL_ADMIN_IP()


if __name__ == '__main__':
    unittest.main()
