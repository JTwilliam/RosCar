#!/usr/bin/env python3
"""Read-only health diagnostics display; pure state/format helpers need no ROS."""
import math
import sys
import time

PREFIX = 'learm/health/'
COMPONENTS = ('link', 'chassis', 'arm', 'communication', 'safety', 'resources', 'tasks')
LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
UNKNOWN = '未知'


def validate_parameters(stale_after_s, offline_after_s, refresh_hz):
    values = (stale_after_s, offline_after_s, refresh_hz)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('stale_after_s, offline_after_s and refresh_hz must be finite positive numbers')
    if offline_after_s <= stale_after_s:
        raise ValueError('offline_after_s must be greater than stale_after_s')
    return values


def clean(value):
    if value is None or str(value).strip() == '':
        return UNKNOWN
    # Diagnostic strings must not inject terminal controls or extra output lines.
    return ''.join(c if c.isprintable() else '?' for c in str(value))


def nonnegative(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) and number >= 0 else None


def csv_five(value):
    parts = str(value).split(',') if value is not None else []
    return [nonnegative(parts[i].strip()) if i < len(parts) else None for i in range(5)]


def number_text(value, suffix=''):
    return UNKNOWN if value is None else f'{value:g}{suffix}'


def level_number(value):
    # Humble diagnostic_msgs uses byte (bytes in Python), other bindings use int.
    if isinstance(value, bytes):
        return value[0] if len(value) == 1 else None
    return value if isinstance(value, int) and not isinstance(value, bool) else None


class HealthState:
    def __init__(self, stale_after_s=0.5, offline_after_s=2.0, refresh_hz=5.0):
        self.stale_after_s, self.offline_after_s, self.refresh_hz = validate_parameters(
            stale_after_s, offline_after_s, refresh_hz)
        self.entries = {}

    def update(self, statuses, received_at):
        """Consume DiagnosticStatus-like objects, retaining only recognized names."""
        for status in statuses:
            name = getattr(status, 'name', '')
            if not name.startswith(PREFIX):
                continue
            component = name[len(PREFIX):]
            if component not in COMPONENTS:
                continue
            # Replace all fields: omitted fields in a new message become unknown.
            values = {str(kv.key): str(kv.value) for kv in getattr(status, 'values', [])}
            self.entries[component] = {
                'values': values, 'level': level_number(getattr(status, 'level', None)),
                'message': getattr(status, 'message', ''), 'received_at': received_at,
            }

    def ages(self, now):
        entry = self.entries.get('link')
        if entry is None:
            return None, None
        diagnostic_age = max(0.0, now - entry['received_at'])
        upstream_age = nonnegative(entry['values'].get('health_age_s'))
        return diagnostic_age, None if upstream_age is None else upstream_age + diagnostic_age

    def link_state(self, now):
        diagnostic_age, health_age = self.ages(now)
        if diagnostic_age is None:
            return 'OFFLINE'
        # Either silent bridge or silent firmware must independently degrade link.
        maximum = max(diagnostic_age, health_age if health_age is not None else 0.0)
        reported = clean(self.entries['link'].get('message')).strip().upper()
        if maximum >= self.offline_after_s or reported == 'OFFLINE':
            return 'OFFLINE'
        if maximum >= self.stale_after_s or reported == 'STALE' or self.entries['link']['level'] == 3:
            return 'STALE'
        return 'ONLINE' if health_age is not None else 'UNKNOWN'


def format_health(state, now):
    """Return a text snapshot without terminal escapes, I/O or inferred states."""
    link = state.entries.get('link', {})
    values = link.get('values', {})
    get = lambda key: clean(values.get(key))
    bridge_age, health_age = state.ages(now)
    receive_age = nonnegative(values.get('receive_age_s'))
    if receive_age is not None and bridge_age is not None:
        receive_age += bridge_age
    lines = [f'RosCar Health | Link {state.link_state(now)}',
             '健康快照年龄: ' + number_text(health_age, 's') +
             ' | 帧接收年龄: ' + number_text(receive_age, 's') +
             ' | link诊断年龄: ' + number_text(bridge_age, 's'),
             f'健康协议: {get("schema")} | 快照时间戳(ms): {get("health_timestamp_ms")}',
             f'固件: {get("firmware_id")} | uptime_ms: {get("uptime_ms")}',
             f'安全状态: {get("safety_state")} | owner: {get("owner")} | event: {get("last_event")}',
             f'当前故障: {get("active_faults")} | 历史故障: {get("historical_faults")}',
             f'最近故障: {get("last_fault")} | last_fault_age_s: {get("last_fault_age_s")}',
             '组件（健康等级与安全状态独立；字段为最近上报值）:']
    for component in COMPONENTS:
        entry = state.entries.get(component, {})
        level = LEVELS.get(entry.get('level'), UNKNOWN)
        fields = entry.get('values', {})
        diagnostic_age = None if not entry else max(0.0, now - entry['received_at'])
        if entry and (state.link_state(now) in ('STALE', 'OFFLINE') or
                      diagnostic_age >= state.stale_after_s):
            level = 'STALE'
        lines.append(f'  {component:13} 健康={level:5} detail={clean(fields.get("detail"))}'
                     f' | 诊断年龄={number_text(diagnostic_age, "s")}')
    tasks = state.entries.get('tasks', {}).get('values', {})
    heartbeats = csv_five(tasks.get('heartbeat_age_ms'))
    stacks = csv_five(tasks.get('stack_free_words'))
    task_names = ['Default', 'Chassis', 'Serial', 'Arm', 'Health']
    supplied_names = tasks.get('task_names', '').split(',')
    if len(supplied_names) == 5 and all(name.strip() for name in supplied_names):
        task_names = [clean(name.strip()) for name in supplied_names]
    lines.append('任务（固件顺序；数值为最近上报值，不推测阈值）:')
    for index, (heartbeat, stack) in enumerate(zip(heartbeats, stacks), 1):
        lines.append(f'  {task_names[index - 1]} (task{index}): heartbeat_age_ms={number_text(heartbeat)}'
                     f' | stack_free_words={number_text(stack)}')
    available = [value for value in stacks if value is not None]
    if len(available) == 5:
        minimum = number_text(min(available))
    elif available:
        minimum = UNKNOWN + '（已知项最小=' + number_text(min(available)) + '）'
    else:
        minimum = UNKNOWN
    lines.append('5任务最小剩余栈(words): ' + minimum)
    lines.append('Ctrl+C退出；ONLINE仅表示报文新鲜，不代表无故障或安全可运行。')
    return chr(10).join(lines)


def main(args=None):
    # Import ROS only at runtime, keeping pure unittest independent of ROS.
    import rclpy
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray

    class HealthMonitor(Node):
        def __init__(self):
            super().__init__('roscar_health_monitor')
            stale = self.declare_parameter('stale_after_s', 0.5).value
            offline = self.declare_parameter('offline_after_s', 2.0).value
            refresh = self.declare_parameter('refresh_hz', 5.0).value
            self.state = HealthState(stale, offline, refresh)
            self.subscription = self.create_subscription(
                DiagnosticArray, '/diagnostics', self.receive, 10)
            self.is_tty = sys.stdout.isatty()
            self.last_print_at = None
            # Output is wall/monotonic paced even when ROS simulated time stops.

        def receive(self, message):
            self.state.update(message.status, time.monotonic())

        def refresh(self):
            now = time.monotonic()
            period = 1.0 / self.state.refresh_hz
            if not self.is_tty:
                period = max(1.0, period)
            if self.last_print_at is None or now - self.last_print_at >= period:
                text = format_health(self.state, now)
                if self.is_tty:
                    sys.stdout.write(chr(27) + '[2J' + chr(27) + '[H')
                print(text, flush=True)
                self.last_print_at = now

    node = None
    rclpy.init(args=args)
    try:
        node = HealthMonitor()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.1, 1.0 / node.state.refresh_hz))
            node.refresh()
    except KeyboardInterrupt:
        pass
    except (BrokenPipeError, OSError):
        # A closed pipe is a normal non-interactive consumer shutdown.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
