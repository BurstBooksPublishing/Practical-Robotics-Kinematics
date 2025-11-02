# Quick decision tool (prototype). Scores 0..10 per metric.
candidates = ["moveit", "pinocchio", "drake", "kdl", "orocos"]
# Example weights: higher weight favors real-time and integration.
weights = {"realtime": 5, "throughput": 4, "integration": 3, "support": 2}

# Example scored capabilities (engineer-assigned or benchmarked).
scores = {
    "moveit":    {"realtime": 0, "throughput": 2, "integration": 8, "support": 9},
    "pinocchio": {"realtime": 7, "throughput": 9, "integration": 6, "support": 7},
    "drake":     {"realtime": 4, "throughput": 8, "integration": 5, "support": 6},
    "kdl":       {"realtime": 8, "throughput": 6, "integration": 7, "support": 5},
    "orocos":    {"realtime": 9, "throughput": 7, "integration": 8, "support": 4},
}

def weighted_score(lib):
    num = sum(weights[k]*scores[lib][k] for k in weights)
    den = sum(weights.values())
    return num/den

# Capability probe: check quick importability (optional).
availability = {}
for lib in candidates:
    try:
        if lib == "pinocchio":
            import pinocchio as pin
        elif lib == "moveit":
            import moveit_commander as mc
        elif lib == "drake":
            from pydrake.all import MathematicalProgram  # probe
        elif lib == "kdl":
            import PyKDL as kdl
        elif lib == "orocos":
            import rtt  # Orocos RTT python bindings
        availability[lib] = True
    except Exception:
        availability[lib] = False

ranked = sorted(candidates, key=lambda l: (availability[l], weighted_score(l)), reverse=True)
for lib in ranked:
    print(f"{lib}: available={availability[lib]}, score={weighted_score(lib):.2f}")
# Use the ranked list plus hard filters for final selection.