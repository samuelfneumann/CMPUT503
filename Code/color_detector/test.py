import numpy as np


N_SPLITS = 5
BINS = 7
MAX = 5

a = np.random.randint(0, 255, (15, 10, 3))
step = a.shape[0] // N_SPLITS


def convert_to_range(hist):
    return (
        np.round(
            (hist[0].max() - hist[0]) / (hist[0].max() - hist[0].min()) * MAX,
        ),
        hist[1],
    )


def print_hist(hist, title=""):
    height, bins = hist
    lines = []

    # Create the title
    length = (len(height) + 1) * 2
    spaces = (length - len(title)) // 2
    rem = length - (spaces * 2 + len(title))
    lines.append(" " * spaces + title + " " * spaces + " " * rem)

    lines.append("⎢" + " " * (length - 2))

    for i in range(MAX, 0, -1):
        line = "⎢"

        for j in range(len(height)):
            if height[j] < i:
                line += "  "
            else:
                line += "█ "
        lines.append(line)

    lines.append("__" * (len(bins)))
    return lines


for i in range(0, (a.shape[0] // N_SPLITS) - 1):
    img = a[i:i+step, :, :]
    r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]

    r_hist = np.histogram(r, range=(0, 255), bins=BINS)
    g_hist = np.histogram(g, range=(0, 255), bins=BINS)
    b_hist = np.histogram(b, range=(0, 255), bins=BINS)

    r_hist = convert_to_range(r_hist)
    g_hist = convert_to_range(g_hist)
    b_hist = convert_to_range(b_hist)

    r_lines = print_hist(r_hist, title="Red")
    g_lines = print_hist(g_hist, title="Green")
    b_lines = print_hist(b_hist, title="Blue")

    lines = map(lambda x: x[0] + x[1] + x[2], zip(r_lines, g_lines, b_lines))

    lines = list(lines)
    for i in range(len(lines)):
        print(r_lines[i], end="    ")
        print(g_lines[i], end="    ")
        print(b_lines[i])
