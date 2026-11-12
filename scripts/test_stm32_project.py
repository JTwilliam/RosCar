#!/usr/bin/env python3
"""Regression checks for a portable, startup-correct Keil source distribution."""
from pathlib import Path
import shutil
import tempfile
import unittest

from check_stm32_project import DEFAULT_SOURCE, c_function, validate_project


class ProjectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='roscar-keil-check-')
        cls.source = Path(cls.temporary.name) / 'App'
        shutil.copytree(DEFAULT_SOURCE, cls.source,
                        ignore=shutil.ignore_patterns('build', '__pycache__'))

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def change(self, relative, old, new):
        path = self.source / relative
        original = path.read_bytes()
        text = original.decode()
        self.assertIn(old, text)
        path.write_bytes(text.replace(old, new, 1).encode())
        self.addCleanup(path.write_bytes, original)

    def test_all_project_dependencies_are_local(self):
        self.assertEqual(validate_project(self.source)['project_inputs'], 148)

    def test_health_init_in_comment_does_not_count(self):
        self.change('Core/Src/main.c', 'robot_health_init(HAL_GetTick());',
                    '/* robot_health_init(HAL_GetTick()); */')
        with self.assertRaisesRegex(ValueError, 'missing startup call: robot_health_init'):
            validate_project(self.source)

    def test_missing_safety_init_rejects_previous_candidate(self):
        self.change('Core/Src/freertos.c', 'robot_safety_init(HAL_GetTick())',
                    'uninitialized_safety(HAL_GetTick())')
        with self.assertRaisesRegex(ValueError, 'missing startup call: robot_safety_init'):
            validate_project(self.source)

    def test_late_safety_init_is_rejected(self):
        self.change('Core/Src/freertos.c', 'robot_safety_init(HAL_GetTick())',
                    'osThreadNew(NULL, NULL, NULL) && robot_safety_init(HAL_GetTick())')
        with self.assertRaisesRegex(ValueError, 'invalid startup order'):
            validate_project(self.source)

    def test_unchecked_safety_failure_is_rejected(self):
        self.change('Core/Src/freertos.c', 'Error_Handler();', '(void)0;')
        with self.assertRaisesRegex(ValueError, 'initialization failure'):
            validate_project(self.source)

    def test_bootloader_overlap_is_rejected(self):
        self.change('MDK-ARM/LeArm/LeArm.sct', 'LR_IROM1 0x08004000',
                    'LR_IROM1 0x08000000')
        with self.assertRaisesRegex(ValueError, 'App scatter'):
            validate_project(self.source)

    def test_external_project_input_is_rejected(self):
        self.change('MDK-ARM/LeArm.uvprojx',
                    '<FilePath>startup_stm32f103xb.s</FilePath>',
                    '<FilePath>../../outside.s</FilePath>')
        with self.assertRaisesRegex(ValueError, 'leaves App directory'):
            validate_project(self.source)

    def test_parser_handles_nested_blocks_and_comments(self):
        body = c_function('/* main() { bad(); } */ int main(void) '
                          '{ if (1) { first(); } second(); }', 'main')
        self.assertNotIn('bad', body)
        self.assertIn('second()', body)


if __name__ == '__main__':
    unittest.main()
