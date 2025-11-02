import subprocess, random, time

ROBOTS = ["robot01.example", "robot02.example", "robot03.example"]  # sample list
BUNDLE = "/tmp/update.raucb"  # signed bundle prepared by CI
SMOKE_LAUNCH = "/opt/ros_ws/launch/moveit_smoke.launch.py"  # checks a pick/place plan
HEALTH_METRICS = ["joint_controller:", "plan_success:"]  # strings to look for in logs

def ssh(cmd, host):
    return subprocess.run(["ssh", host, cmd], check=False, capture_output=True, text=True)

def install_bundle(host):
    # install bundle atomically with RAUC; RAUC validates signature and writes A/B partition
    out = ssh(f"sudo rauc install {BUNDLE}", host)
    return out.returncode == 0

def run_smoke(host):
    # run a short MoveIt test; use timeout to avoid hangs
    out = ssh(f"timeout 30 ros2 launch {SMOKE_LAUNCH}", host)
    logs = out.stdout + out.stderr
    # quick health detection: look for expected strings
    return all(s in logs for s in HEALTH_METRICS)

def rollback(host):
    # instruct bootloader/system to revert to previous slot
    ssh("sudo rauc rollback", host)

# basic canary procedure
sample = random.sample(ROBOTS, k=min(3, len(ROBOTS)))  # small sample example
for r in sample:
    if not install_bundle(r):
        rollback(r); raise SystemExit(f"Install failed on {r}")
    # allow controllers to come up before testing
    time.sleep(5)
    if not run_smoke(r):
        rollback(r); raise SystemExit(f"Smoke test failed on {r}")

# if all pass, promote to next cohort (not implemented)
print("Canary cohort healthy; promote update.")