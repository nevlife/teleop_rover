import time

from teleop_contracts import (
    TOPIC_CPU,
    TOPIC_DISK,
    TOPIC_ESTOP_STATUS,
    TOPIC_GPU,
    TOPIC_MEM,
    TOPIC_MUX,
    TOPIC_NET,
    TOPIC_TWIST,
    key_for,
)


class TelemetrySerializer:

    @staticmethod
    def serialize_vehicle(
        vehicle_id: str,
        mux_status,
        last_nav,
        last_teleop,
        last_final,
        current_speed: float,
        current_steer_angle: float,
        estop_status,
    ) -> dict[str, dict]:
        ts = time.time()
        payloads = {}

        # NOTE on the per-payload `ts`: predates the envelope and is kept
        # for back-compat (server.parser still reads it for tele_delay_ms).
        # The envelope's own `ts` is added later by ZenohTransport.put.
        payloads[key_for(vehicle_id, TOPIC_MUX)] = {
            "ts": ts,
            "requested_mode": int(mux_status.mode),
            "active_source": int(mux_status.cmd_source),
            "remote_enabled": bool(mux_status.remote_status),
            "nav_active": bool(mux_status.nav_active),
            "teleop_active": bool(mux_status.teleop_active),
            "final_active": bool(mux_status.final_active),
        }

        payloads[key_for(vehicle_id, TOPIC_TWIST)] = {
            "ts": ts,
            "nav_lx": float(last_nav.linear.x),
            "nav_az": float(last_nav.angular.z),
            "teleop_lx": float(last_teleop.linear.x),
            "teleop_az": float(last_teleop.angular.z),
            "final_lx": float(last_final.linear.x),
            "final_az": float(last_final.angular.z),
            "current_speed": float(current_speed),
            "current_steer_angle": float(current_steer_angle),
        }

        payloads[key_for(vehicle_id, TOPIC_ESTOP_STATUS)] = {
            "ts": ts,
            "is_estop": bool(estop_status.is_estop),
            "bridge_flag": int(estop_status.bridge_flag),
            "mux_flag": int(estop_status.mux_flag),
        }

        return payloads

    @staticmethod
    def serialize_resources(vehicle_id: str, cpu, mem, gpu, disk, net) -> dict[str, dict]:
        payloads = {}

        if cpu is not None:
            payloads[key_for(vehicle_id, TOPIC_CPU)] = {
                "cpu_usage": float(cpu.usage_percent),
                "cpu_temp": float(cpu.temperature_celsius),
                "cpu_load": float(cpu.load_avg_1m),
            }

        if mem is not None:
            payloads[key_for(vehicle_id, TOPIC_MEM)] = {
                "ram_total": int(mem.total_bytes // (1024 * 1024)),
                "ram_used": int(mem.used_bytes // (1024 * 1024)),
            }

        if gpu is not None:
            # GPU was historically a bare JSON list. Wrapped under "gpus"
            # so the envelope (which must be a JSON object) can carry the
            # {v, ts} fields uniformly. Server reads data["gpus"].
            payloads[key_for(vehicle_id, TOPIC_GPU)] = {
                "gpus": [
                    {
                        "idx": int(g.index),
                        "gpu_usage": float(g.utilization_percent),
                        "gpu_mem_used": float(g.memory_used_mb),
                        "gpu_mem_total": float(g.memory_total_mb),
                        "gpu_temp": float(g.temperature_celsius),
                        "gpu_power": float(g.power_watts),
                    }
                    for g in gpu.gpus
                ]
            }

        if disk is not None:
            payloads[key_for(vehicle_id, TOPIC_DISK)] = {
                "partitions": [
                    {
                        "idx": i,
                        "mountpoint": p.mountpoint,
                        "total_bytes": int(p.total_bytes),
                        "used_bytes": int(p.used_bytes),
                        "percent": float(p.percent),
                        "accessible": bool(p.accessible),
                    }
                    for i, p in enumerate(disk.partitions)
                ]
            }

        if net is not None:
            payloads[key_for(vehicle_id, TOPIC_NET)] = {
                "net_total_ifaces": int(net.total_interfaces),
                "net_active_ifaces": int(net.active_interfaces),
                "net_down_ifaces": int(net.down_interfaces),
                "interfaces": [
                    {
                        "idx": i,
                        "name": iface.name,
                        "is_up": bool(iface.is_up),
                        "speed_mbps": int(iface.speed_mbps),
                        "in_bps": float(iface.input_bytes_per_sec),
                        "out_bps": float(iface.output_bytes_per_sec),
                    }
                    for i, iface in enumerate(net.interfaces)
                ],
            }

        return payloads
