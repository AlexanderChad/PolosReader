import time
import cv2
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import filedialog
import numpy as np
import math

img: np.array  # загруженное изображение
display_img: np.array  # выводимое на дисплей изображение
img_label: tk.Label  # объект, на который выводится изображение
lines_c = []  # координаты центров линий по оси X

adj_threshold: tk.Scale  # объект ползунка threshold (порог для маски)
# объект ползунка dopusk (смешивание соседних полос в линии)
adj_dopusk: tk.Scale
adj_overlap: tk.Scale  # объект ползунка overlap (перекрытие)
# объект ползунка gain_bg (усиление фона, т.е. вне полос)
adj_gain_bg: tk.Scale
adj_gain_line: tk.Scale  # объект ползунка gain_line (усиление линий, в белый)
adj_brightness: tk.Scale  # объект ползунка brightness (яркость)
adj_contrast: tk.Scale  # объект ползунка contrast (контрастность)

height = 0  # высота загрженного изображения
width = 0  # ширина загрженного изображения
line_width = 0  # средняя ширина линий

first_int = True  # флаг первого запуска. Добавляет задержку между этапами

root = tk.Tk()  # используем графический интерфейс Tkinter
root.wm_title("ImgMagic")


def UpdImage(Show_img, BC=True):  # Вывод изображения на экран
    global img_label, adj_brightness, adj_contrast, display_img
    if BC:  # если необходимо применить коррекцию яркости и контрастности
        img_upgraded = apply_brightness_contrast(
            Show_img, adj_brightness.get(), adj_contrast.get())  # применяем настройки яркости и контрастности
    else:
        img_upgraded = np.copy(Show_img)
    converted_img = Image.fromarray(img_upgraded)  # конвертируем в формат PIL
    converted_img.thumbnail(size=(809, 688))  # подгоняем размер
    imgtk = ImageTk.PhotoImage(image=converted_img)  # конвертируем в формат TK
    img_label.imgtk = imgtk  # присваиваем
    img_label.configure(image=imgtk)  # обновляем
    print('Img updated')
    root.update()  # обновляем UI
    # сохраняем в буфер для возможности позднее сохранить в файл
    display_img = np.copy(img_upgraded)
    if first_int:  # если первый проход
        time.sleep(3)  # между обновлениями делаем задержку


def FindLines():  # Ищем линии
    global lines_c, line_width
    # Обнуляем
    lines_c = []
    contours = []
    lines_w = []
    # выделяем по порогу маску
    _, threshold = cv2.threshold(
        img, adj_threshold.get(), 255, cv2.THRESH_BINARY)
    # обновляем изображение
    UpdImage(cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB))

    # получаем контуры палочек
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # конвертируем в пространство RGB из тонов серого
    img2 = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    # игнорием первый контур, он для нас побочный
    contours = contours[1:]

    # перебираем контуры
    for contour in contours:
        # рисуем найденный контур на изображении
        cv2.drawContours(img2, [contour], 0, (255, 0, 0), 2)

        # вычисляем с помощью моментов центр контура
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        '''
        # старый метод нахождения краев линии
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img2, [box], 0, (0, 0, 255), 2)
        x, y, w, h = cv2.boundingRect(box)
        '''
        # новый метод нахождения краев линии
        xl = []
        xr = []
        # получаем средний x и длину отрезка
        con_p = [[(contour[i-1][0][0]+contour[i][0][0])/2,
                  math.dist(contour[i-1][0], contour[i][0])] for i in range(len(contour))]
        for cnp in con_p:  # перебираем отрезки
            if cnp[0] < cX:  # если левый, то записываем в левый
                xl.append(cnp)
            elif cnp[0] > cX:  # если правый, то записываем в правый
                xr.append(cnp)
        # вычисляем средний X, определяя за вес длину отрезка
        xlc = sum(x[0]*x[1] for x in xl)/sum(x[1] for x in xl)
        xrc = sum(x[0]*x[1] for x in xr)/sum(x[1] for x in xr)
        # вычисляем ширину линии
        w = xrc-xlc

        # записываем вычисленные параметры контура (линии)
        lines_c.append(cX)
        lines_w.append(w)
        # рисуем линию
        cv2.line(img2, (cX, 0), (cX, height),
                 (0, 255, 0), thickness=2)
    # сортируем линии по координатам
    lines_c.sort()
    # вычисляем среднюю ширину линии
    line_width = round(sum(lines_w)/len(lines_w))
    # обновляем изображение
    UpdImage(img2, False)


# коррекция яркости и контрастности изображения
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


def RestoreImg():  # восстанавливаем изображение
    global lines_c, line_width
    # инвертируем изображение
    img_res = 255-np.copy(img)
    # получаем половину расстояния между соседними линиями
    dist_line = round((lines_c[-1]-lines_c[0])/(len(lines_c)-1)/2)
    # получаем среднюю половину ширины линии
    width_line = round(line_width/2)
    # считываем ползунки
    gain_bg = adj_gain_bg.get()/100
    gain_line = adj_gain_line.get()/100
    dopusk = adj_dopusk.get()/100
    overlap = adj_overlap.get()/100
    # считаем ширину перекрытия
    overlap_width = round(overlap*dist_line)
    # цвет прошлого прохода
    prev_line_right_color = 0
    # перебираем линии
    for line in lines_c:
        # для левой стороны линии
        llp = line-dist_line  # левая граница левого сектора
        lrp = line-width_line  # правая граница левого сектора
        # для правой стороны линии
        rlp = line+width_line  # левая граница правого сектора
        rrp = line+dist_line  # правая граница правого сектора
        # проходимся по высоте
        for j in range(height):
            # для левой стороны линии
            if llp >= 0:  # проверяем что есть место для работы
                color_l = round(sum([img_res[j, i]
                                     for i in range(llp, lrp)])/(lrp-llp))
            else:
                continue
            # для правой стороны линии
            if rrp <= width:  # проверяем что есть место для работы
                color_r = round(sum([img_res[j, i]
                                     for i in range(rlp, rrp)])/(rrp-rlp))
            else:
                continue
            # считаем средний цвет пары секторов (левый и правый)
            delta_color = (color_l-color_r)/2
            # обновляем основной цвет для левого
            color_l = round(color_l-delta_color*dopusk)
            # обновляем основной цвет для правого
            color_r = round(color_r+delta_color*dopusk)
            # восстанавливаем левый сектор
            for i in range(llp, lrp):
                img_res[j, i] = round(
                    img_res[j, i]*gain_bg+color_l*(1-gain_bg))
            # восстанавливаем правый сектор
            for i in range(rlp, rrp):
                img_res[j, i] = round(
                    img_res[j, i]*gain_bg+color_r*(1-gain_bg))
            # считаем основной цвет на основании крайних пикселей
            color_c = round((img_res[j, lrp]+img_res[j, rlp])/2)
            # восстанавливаем центральный сектор (область линии)
            for i in range(lrp, rlp):
                img_res[j, i] = round(color_c*(1-gain_line))
            # применяем перекрытие в месте стыковки секторов линий
            for i in range(llp-overlap_width, llp+overlap_width):
                img_res[j, i] = round((prev_line_right_color+color_l)/2)
            # запоминаем цвет линии
            prev_line_right_color = color_r
    print('Success')
    # обновляем изображение (не забываем инвертировать обратно)
    UpdImage(cv2.cvtColor(255-img_res, cv2.COLOR_GRAY2RGB))


def ApplyProp():  # Применяем настройки и перестраиваем изображение
    UpdImage(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB), False)
    FindLines()
    RestoreImg()


def SaveImg():  # Сохраняем изображение
    global display_img
    img_name = time.strftime("%d-%m-%Y_%H.%M.%S", time.localtime()) + \
        f'_t{adj_threshold.get()},_d{adj_dopusk.get()},_o{adj_overlap.get()},_gbg' + \
        f'{adj_gain_bg.get()},_gl{adj_gain_line.get()},_b{adj_brightness.get()},_c{adj_contrast.get()}.png'
    cv2.imwrite(img_name, display_img)


if __name__ == "__main__":
    # вызываем диалог открытия изображения
    img_path = filedialog.askopenfilename(filetypes=(
        ("Image files", "*.png;*.jpg;*.jpeg;*.bmp"), ("All files", "*.*")))

    # читаем изображение
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    # читаем его размеры
    height, width = img.shape

    # прописываем UI Tk
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
    label_adj_overlap.grid(row=4, column=0)
    adj_overlap = tk.Scale(root, length=250, from_=0,
                           to=100, orient=tk.HORIZONTAL)
    adj_overlap.grid(row=4, column=1)

    label_adj_gain_bg = tk.Label(root, text="adj_gain_bg, %")
    label_adj_gain_bg.grid(row=5, column=0)
    adj_gain_bg = tk.Scale(root, length=250, from_=0,
                           to=100, orient=tk.HORIZONTAL)
    adj_gain_bg.grid(row=5, column=1)

    label_adj_gain_line = tk.Label(root, text="adj_gain_line, %")
    label_adj_gain_line.grid(row=6, column=0)
    adj_gain_line = tk.Scale(root, length=250, from_=0,
                             to=100, orient=tk.HORIZONTAL)
    adj_gain_line.grid(row=6, column=1)

    label_adj_brightness = tk.Label(root, text="adj_brightness")
    label_adj_brightness.grid(row=7, column=0)
    adj_brightness = tk.Scale(
        root, length=250, from_=-127, to=127, orient=tk.HORIZONTAL)
    adj_brightness.grid(row=7, column=1)

    label_adj_contrast = tk.Label(root, text="adj_contrast")
    label_adj_contrast.grid(row=8, column=0)
    adj_contrast = tk.Scale(root, length=250, from_=-
                            127, to=127, orient=tk.HORIZONTAL)
    adj_contrast.grid(row=8, column=1)

    # настройки по-умолчанию
    adj_threshold.set(127)
    adj_dopusk.set(0)
    adj_overlap.set(0)
    adj_gain_bg.set(50)
    adj_gain_line.set(50)
    adj_brightness.set(0)
    adj_contrast.set(0)

    # запускаем перестроение
    ApplyProp()

    # отмечаем, что один раз уже строили
    first_int = False

    # отдаем управление UI
    root.mainloop()
