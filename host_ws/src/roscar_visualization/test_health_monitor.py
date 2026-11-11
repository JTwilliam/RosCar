"""Pure stdlib unittest; importing the monitor does not import ROS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest

spec = importlib.util.spec_from_file_location(
    'health_monitor', Path(__file__).parent / 'scripts/health_monitor.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def status(component, level=0, **values):
    return Obj(name=monitor.PREFIX + component, level=level, message='',
               values=[Obj(key=key, value=value) for key, value in values.items()])


class HealthTest(unittest.TestCase):
    def test_ok_diagnostic_still_times_out(self):
        state = monitor.HealthState()
        state.update([status('link', health_age_s='0.1')], 10.0)
        self.assertEqual(state.link_state(10.0), 'ONLINE')
        self.assertEqual(state.link_state(10.4), 'STALE')
        self.assertEqual(state.link_state(12.0), 'OFFLINE')
        self.assertEqual(state.entries['link']['level'], 0)

    def test_upstream_age_independently_degrades(self):
        state = monitor.HealthState()
        state.update([status('link', health_age_s='0.6')], 1.0)
        self.assertEqual(state.link_state(1.0), 'STALE')
        state.update([status('link', health_age_s='2.1')], 2.0)
        self.assertEqual(state.link_state(2.0), 'OFFLINE')

    def test_unrelated_and_component_diagnostics_do_not_refresh_link(self):
        state = monitor.HealthState()
        state.update([status('link', health_age_s='0')], 1.0)
        unrelated = status('link', health_age_s='0')
        unrelated.name = 'learm_serial_bridge'
        state.update([unrelated, status('arm', detail='IDLE'),
                      status('unknown', detail='ignore')], 4.0)
        self.assertEqual(state.link_state(4.0), 'OFFLINE')
        self.assertNotIn('unknown', state.entries)
        self.assertEqual(state.entries['link']['received_at'], 1.0)

    def test_missing_fields_replace_instead_of_retaining_old_values(self):
        state = monitor.HealthState()
        self.assertEqual(state.link_state(0), 'OFFLINE')
        state.update([status('link', health_age_s='0', firmware_id='abc')], 1)
        state.update([status('link')], 2)
        self.assertEqual(state.link_state(2), 'UNKNOWN')
        self.assertIsNone(state.ages(2)[1])
        self.assertNotIn('abc', monitor.format_health(state, 2))
        self.assertEqual(state.link_state(2.5), 'STALE')
        self.assertEqual(state.link_state(4), 'OFFLINE')

    def test_invalid_age_is_unknown(self):
        for age in ('nan', 'inf', '-1', '', 'bad'):
            with self.subTest(age=age):
                state = monitor.HealthState()
                state.update([status('link', health_age_s=age)], 0)
                self.assertEqual(state.link_state(0), 'UNKNOWN')

    def test_format_preserves_safety_health_and_faults(self):
        state = monitor.HealthState()
        state.update([status('link', level=b'\x02', health_age_s='0',
                             firmware_id='fw42', safety_state='ESTOP', owner='HOST',
                             active_faults='0x0001', historical_faults='0x0081',
                             last_event='STOP', last_fault='OVERCURRENT'),
                      status('arm', detail='IDLE'),
                      status('tasks', heartbeat_age_ms='1,2,3,4,5',
                             stack_free_words='100,80,120,90,110')], 10)
        text = monitor.format_health(state, 10.1)
        self.assertIn('Link ONLINE', text)
        self.assertIn('安全状态: ESTOP', text)
        self.assertIn('健康=ERROR', text)
        self.assertIn('detail=IDLE', text)
        self.assertIn('当前故障: 0x0001 | 历史故障: 0x0081', text)
        self.assertIn('5任务最小剩余栈(words): 80', text)
        self.assertIn('Health (task5): heartbeat_age_ms=5', text)
        self.assertIn('chassis', text)
        self.assertIn('未知', text)
        self.assertNotIn('\x1b', text)

    def test_partial_task_fields_do_not_invent_minimum(self):
        state = monitor.HealthState()
        state.update([status('tasks', heartbeat_age_ms='1,,oops',
                             stack_free_words='50,30')], 0)
        text = monitor.format_health(state, 1)
        self.assertIn('Health (task5): heartbeat_age_ms=未知 | stack_free_words=未知', text)
        self.assertIn('5任务最小剩余栈(words): 未知（已知项最小=30）', text)
        self.assertEqual(monitor.csv_five('1,2,3,4,5,6'), [1,2,3,4,5])

    def test_terminal_controls_are_not_forwarded(self):
        state = monitor.HealthState()
        state.update([status('link', firmware_id='a\x1b[2J\nb')], 0)
        text = monitor.format_health(state, 0)
        self.assertNotIn('\x1b', text)
        self.assertIn('a?[2J?b', text)

    def test_bridge_reported_link_and_component_staleness(self):
        state = monitor.HealthState()
        link = status('link', health_age_s='0')
        link.message = 'OFFLINE'
        state.update([link, status('arm', detail='IDLE')], 0)
        self.assertEqual(state.link_state(0), 'OFFLINE')
        self.assertIn('arm           健康=STALE', monitor.format_health(state, 0))
        link.message = 'STALE'
        state.update([link], .1)
        self.assertEqual(state.link_state(.1), 'STALE')
        link.message = 'ONLINE'
        state.update([link], .6)
        self.assertEqual(state.link_state(.6), 'ONLINE')
        self.assertIn('arm           健康=STALE', monitor.format_health(state, .6))
        state.update([status('link', level=b'\x03')], 1)
        self.assertEqual(state.link_state(1), 'STALE')

    def test_task_names_override(self):
        state = monitor.HealthState()
        state.update([status('tasks', task_names='A,B,C,D,E')], 0)
        self.assertIn('E (task5)', monitor.format_health(state, 0))

    def test_parameters(self):
        self.assertEqual(monitor.validate_parameters(.5, 2, 5), (.5, 2, 5))
        for values in ((0,2,5), (.5,.5,5), (2,.5,5), (.5,2,-1),
                       (float('nan'),2,5), (.5,float('inf'),5), (.5,2,True)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                monitor.HealthState(*values)


if __name__ == '__main__':
    unittest.main(verbosity=2)
