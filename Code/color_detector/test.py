#TODO: detect white, red, and yellow

import numpy as np
import scipy.stats as st


N_SPLITS = 3  # This needs to be fixed
BINS = 10
MAX = 5

if MAX < 5:
    raise ValueError("MAX should be > 5")

a = np.random.randint(0, 255, (64, 48, 3))
step = a.shape[0] // N_SPLITS

if N_SPLITS > a.shape[0]:
    raise ValueError("N_SPLITS should be <= image height")


def convert_to_range(hist):
    return (
        np.round(
            (hist[0].max() - hist[0]) / (hist[0].max() - hist[0].min()) * MAX,
        ),
        hist[1],
    )


def format_title(title):
    if title == "Red":
        return "\x1b[1;31m"
    elif title == "Green":
        return "\x1b[1;32m"
    elif title == "Blue":
        return "\x1b[1;34m"
    else:
        return ""


def print_hist(hist, title=""):
    height, bins = hist
    lines = []

    # Create the title
    length = (len(height) + 1) * 2
    spaces = (length - len(title)) // 2
    rem = length - (spaces * 2 + len(title))
    lines.append(format_title(title) + " " * spaces + title + " " * spaces +
                 " " * rem + "\x1b[1;0m")

    lines.append("⎢" + " " * (length - 2))

    for i in range(MAX, 0, -1):
        line = "⎢"

        for j in range(len(height)):
            if height[j] < i:
                line += "  "
            else:
                line += "█ "
        lines.append(line)

    lines.append("__" * (len(bins) - 1) + "_")
    return lines


def print_stats(img):
    r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]

    mean_r = np.mean(r)
    median_r = np.median(r)
    mode_r = st.mode(r.ravel(), keepdims=False).mode
    max_r = np.max(r)
    min_r = np.min(r)

    mean_g = np.mean(g)
    median_g = np.median(g)
    mode_g = st.mode(g.ravel(), keepdims=False).mode
    max_g = np.max(g)
    min_g = np.min(g)

    mean_b = np.mean(b)
    median_b = np.median(b)
    mode_b = st.mode(b.ravel(), keepdims=False).mode
    max_b = np.max(b)
    min_b = np.min(b)

    red = format_title("Red")
    green = format_title("Green")
    blue = format_title("Blue")
    lines = (
        f"     {red}   Red   \t{green}Green   {blue}Blue\n\x1b[1;0m",
        "        |------------------------",
        f"Mean    |  {mean_r:3.0f}\t{mean_g:3.0f}\t{mean_b:3.0f}",
        f"Median  |  {median_r:3.0f}\t{median_g:3.0f}\t{median_b:3.0f}",
        f"Mode    |  {mode_r:3.0f}\t{mode_g:3.0f}\t{mode_b:3.0f}",
        f"Max     |  {max_r:3.0f}\t{max_g:3.0f}\t{max_b:3.0f}",
        f"Min     |  {min_r:3.0f}\t{min_g:3.0f}\t{min_b:3.0f}",
    )

    return lines


def print_img_data(im):
    print("======" * 20)
    for j in range(0, N_SPLITS):
        img = im[j:j+step, :, :]
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
        stats = print_stats(img)

        for i in range(len(r_lines)):
            if i == 0:
                print(f"\x1b[1;33mSplit {j+1}\x1b[1;0m", end="   ")
            else:
                print("      " + " " * len(str(j+1)), end="   ")
            print(r_lines[i], end="    ")
            print(g_lines[i], end="    ")
            if i < len(stats):
                print(b_lines[i], end="    ")
                print(stats[i])
            else:
                print(b_lines[i])
        print()

    print("======" * 20)


print_img_data(a)
