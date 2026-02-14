def load_params(filename):
    params = {}

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()

            if not line or "=" not in line:
                continue

            name, value = line.split("=", 1)
            params[name.strip()] = value.strip()

    return params


def compare_param_files(file1, file2):
    p1 = load_params(file1)
    p2 = load_params(file2)

    all_keys = set(p1.keys()) | set(p2.keys())
    changes = []

    for key in sorted(all_keys):
        v1 = p1.get(key)
        v2 = p2.get(key)

        if v1 != v2:
            changes.append((key, v1, v2))

    return changes


# -------- run ----------
changes = compare_param_files("full_param_list.txt", "full_param_list4.txt")

if not changes:
    print("No parameter values changed")
else:
    print("Changed parameters:\n")
    for key, old, new in changes:
        print(f"{key}: {old}  →  {new}")
