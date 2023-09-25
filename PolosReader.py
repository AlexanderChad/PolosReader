import time
import cv2
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import filedialog
import numpy as np

img: np.array
display_img: np.array
img_label: tk.Label
lines_c = []  # lines
adj_threshold: tk.Scale
adj_dopusk: tk.Scale
adj_overlap: tk.Scale
adj_gain_bg: tk.Scale
adj_gain_line: tk.Scale
adj_brightness: tk.Scale
adj_contrast: tk.Scale

height = 0
width = 0

first_int = True

root = tk.Tk()
root.wm_title("ImgMagic")


def UpdImage(Show_img):
    global img_label, adj_brightness, adj_contrast, display_img
    img_upgraded = apply_brightness_contrast(
        Show_img, adj_brightness.get(), adj_contrast.get())
    converted_img = Image.fromarray(img_upgraded)
    converted_img.thumbnail(size=(809, 688))
    imgtk = ImageTk.PhotoImage(image=converted_img)
    img_label.imgtk = imgtk
    img_label.configure(image=imgtk)
    print('Img updated')
    root.update()
    display_img = np.copy(img_upgraded)
    if first_int:
        time.sleep(1)


def FindLines():
    global lines_c
    lines_c = []
    # setting threshold of gray image
    _, threshold = cv2.threshold(
        img, adj_threshold.get(), 255, cv2.THRESH_BINARY)
    # display(Image.fromarray(cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)))
    UpdImage(cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB))

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    img2 = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    # here we are ignoring first counter because
    # findcontour function detects whole image as shape
    contours = contours[1:]

    # list for storing names of shapes
    for contour in contours:
        # using drawContours() function
        cv2.drawContours(img2, [contour], 0, (255, 0, 0), 1)

        # find lines
        x = [contour[i][0][0] for i in range(len(contour))]
        # y=[contour[i][0][1] for i in range(len(contour))]
        x_center = round((min(x)+max(x))/2)
        # [center, 1/2 line width]
        lines_c.append([x_center, round((max(x)-min(x))/2)])
        cv2.line(img2, (x_center, 0), (x_center, height),
                 (0, 255, 0), thickness=1)
        # print(f'x_center={x_center}, y_min={y_min}, y_max={y_max}.')
    lines_c.sort(key=lambda x: x[0])
    UpdImage(img2)


def apply_brightness_contrast(input_img, brightness=0, contrast=0):
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow) / 255
        gamma_b = shadow
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    if contrast != 0:
        f = 131 * (contrast + 127) / (127 * (131 - contrast))
        alpha_c = f
        gamma_c = 127 * (1 - f)
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)
    return buf


def fill_line(img_f: np.array, lx: int, rx: int, y: int, color: int):
    for i in range(lx, rx):
        img_f[i, y] = color


def RestoreImg():
    global lines_c
    img_res = 255-np.copy(img)
    # получаем половину расстояния между соседними линиями
    dist_line = round((lines_c[-1][0]-lines_c[0][0])/(len(lines_c)-1)/2)
    # получаем среднюю половину ширины линии
    width_line = round(sum([lines_c[i][1]
                       for i in range(len(lines_c))])/len(lines_c))
    gain_bg = adj_gain_bg.get()/100
    gain_line = adj_gain_line.get()/100
    dopusk = adj_dopusk.get()/100
    overlap = adj_overlap.get()/100
    overlap_width = round(overlap*dist_line)
    prev_line_right_color = 0
    for line in lines_c:
        # for left
        llp = line[0]-dist_line
        lrp = line[0]-width_line
        # for right
        rlp = line[0]+width_line
        rrp = line[0]+dist_line
        for j in range(height):
            # for left
            if llp >= 0:
                color_l = round(sum([img_res[j, i]
                                     for i in range(llp, lrp)])/(lrp-llp))
            else:
                continue
            # for right
            if rrp <= width:
                color_r = round(sum([img_res[j, i]
                                     for i in range(rlp, rrp)])/(rrp-rlp))
            else:
                continue
            delta_color = (color_l-color_r)/2
            color_l = round(color_l-delta_color*dopusk)
            color_r = round(color_r+delta_color*dopusk)
            # fill_line left
            for i in range(llp, lrp):
                img_res[j, i] = round(
                    img_res[j, i]*gain_bg+color_l*(1-gain_bg))
            # fill_line right
            for i in range(rlp, rrp):
                img_res[j, i] = round(
                    img_res[j, i]*gain_bg+color_r*(1-gain_bg))
            # fill_line center
            color_c = round((img_res[j, lrp]+img_res[j, rlp])/2)
            for i in range(lrp, rlp):
                img_res[j, i] = round(color_c*(1-gain_line))

            for i in range(llp-overlap_width, llp+overlap_width):
                img_res[j, i] = round((prev_line_right_color+color_l)/2)

            prev_line_right_color = color_r

    print('Success')
    UpdImage(cv2.cvtColor(255-img_res, cv2.COLOR_GRAY2RGB))
    # for right


def ApplyProp():
    UpdImage(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))
    FindLines()
    RestoreImg()


def SaveImg():
    global display_img
    img_name = time.strftime("%d-%m-%Y_%H.%M.%S", time.localtime()) + \
        f'_t{adj_threshold.get()},_d{adj_dopusk.get()},_o{adj_overlap.get()},_gbg{adj_gain_bg.get()},_gl{adj_gain_line.get()},_b{adj_brightness.get()},_c{adj_contrast.get()}.png'
    cv2.imwrite(img_name, display_img)


if __name__ == "__main__":
    img_path = filedialog.askopenfilename(filetypes=(
        ("Image files", "*.png;*.jpg;*.jpeg;*.bmp"), ("All files", "*.*")))

    # 'scale_2400.png'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    height, width = img.shape

    # img_frame = tk.Frame(root, width=width, height=height, bg='gray').grid(row=0, column=0)
    img_label = tk.Label(root, bg='gray')
    img_label.grid(row=0, column=0, columnspan=2)

    bt_upd = tk.Button(root, text='ApplyProp', command=ApplyProp).grid(
        row=1, column=0)

    bt_upd = tk.Button(root, text='SaveImg', command=SaveImg).grid(
        row=1, column=1)

    label_adj_threshold = tk.Label(root, text="adj_threshold")
    label_adj_threshold.grid(row=2, column=0)
    adj_threshold = tk.Scale(root, length=250, from_=0,
                             to=255, orient=tk.HORIZONTAL)
    adj_threshold.grid(row=2, column=1)

    label_adj_dopusk = tk.Label(root, text="adj_dopusk, %")
    label_adj_dopusk.grid(row=3, column=0)
    adj_dopusk = tk.Scale(root, length=250, from_=0,
                          to=100, orient=tk.HORIZONTAL)
    adj_dopusk.grid(row=3, column=1)

    label_adj_overlap = tk.Label(root, text="adj_overlap, %")
    label_adj_overlap.grid(row=3, column=0)
    adj_overlap = tk.Scale(root, length=250, from_=0,
                           to=100, orient=tk.HORIZONTAL)
    adj_overlap.grid(row=3, column=1)

    label_adj_gain_bg = tk.Label(root, text="adj_gain_bg, %")
    label_adj_gain_bg.grid(row=4, column=0)
    adj_gain_bg = tk.Scale(root, length=250, from_=0,
                           to=100, orient=tk.HORIZONTAL)
    adj_gain_bg.grid(row=4, column=1)

    label_adj_gain_line = tk.Label(root, text="adj_gain_line, %")
    label_adj_gain_line.grid(row=5, column=0)
    adj_gain_line = tk.Scale(root, length=250, from_=0,
                             to=100, orient=tk.HORIZONTAL)
    adj_gain_line.grid(row=5, column=1)

    label_adj_brightness = tk.Label(root, text="adj_brightness")
    label_adj_brightness.grid(row=6, column=0)
    adj_brightness = tk.Scale(
        root, length=250, from_=-127, to=127, orient=tk.HORIZONTAL)
    adj_brightness.grid(row=6, column=1)

    label_adj_contrast = tk.Label(root, text="adj_contrast")
    label_adj_contrast.grid(row=7, column=0)
    adj_contrast = tk.Scale(root, length=250, from_=-
                            127, to=127, orient=tk.HORIZONTAL)
    adj_contrast.grid(row=7, column=1)

    # def settings
    adj_threshold.set(127)
    adj_dopusk.set(50)
    adj_overlap.set(50)
    adj_gain_bg.set(50)
    adj_gain_line.set(50)
    adj_brightness.set(0)
    adj_contrast.set(0)

    ApplyProp()

    first_int = False

    root.mainloop()

    # img_out = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
    # cv2.imwrite('.\\' + name + '_DQRC.png', img)
