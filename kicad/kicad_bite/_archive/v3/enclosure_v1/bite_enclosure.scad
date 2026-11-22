// =====================================================================
//  BITE SENSOR v3 — корпус для датчика поклёвки на удилище
//  Плата 70 x 24 x 1.0 мм (kicad_bite/bite_sensor.kicad_pcb) сверху,
//  ячейка 18650 снизу, седло с V-канавкой под бланк + 2 ремешка.
//
//  Система координат: начало = центр платы (KiCad 135, 112),
//  X — вдоль корпуса (XIAO слева, зарядный USB-C справа),
//  Y — вверх на виде сверху (= -Y KiCad), Z = 0 — низ корпуса (без седла).
//
//  part = "tub"       — нижняя часть (ячейка, рельсы под плату, седло)
//         "lid"       — крышка (кнопка-язычок, окна светодиодов, зуммер)
//         "assembly"  — сборка с платой и ячейкой для проверки
//         "section"   — разрез сборки по оси кнопки
//         "clash_lid" / "clash_tub" — пересечение с платой (должно быть пусто)
//  Печать: tub — на седле, lid — верхней гранью на стол. PETG, 0.2 мм,
//  4 периметра. Поддержки не нужны.
// =====================================================================
part = "assembly";
sec_x = 3.0;     // положение разреза для part = "section"
$fn = 72;

// ---------- плата и её окружение
pcb_t   = 1.0;          // толщина платы
ix0     = -36.45;       // внутренний торец слева (перед USB XIAO -36.33)
ix1     = 35.25;        // внутренний торец справа (перед J1 35.03)
iy      = 12.35;        // внутренняя полуширина (край платы 12.05)

// ---------- стенки
wall    = 1.6;          // боковые стенки
endw    = 1.2;          // торцы (тонко — чтобы штекер USB-C входил на всю глубину)
bot     = 1.4;          // дно под ячейкой
topw    = 1.4;          // верх крышки

// ---------- ячейка 18650
cell_r  = 9.35;         // радиус ложемента (18.7 мм — с запасом на 18.3…18.6)
zc      = bot + cell_r;              // ось ячейки        10.75
z_pb    = zc + cell_r + 0.1;         // низ платы         20.2
z_pt    = z_pb + pcb_t;              // верх платы        21.2

// ---------- компоненты
comp_h  = 3.5;          // XIAO ESP32-S3 (самый высокий, кроме его USB)
lid_gap = 0.3;
z_li    = z_pt + comp_h + lid_gap;   // потолок крышки    25.0
z_top   = z_li + topw;               // верх корпуса      26.4
z_s     = z_pt;                      // плоскость разъёма = верх платы
tng     = 1.5;                       // глубина шипа крышки в стенку

xiao_pcb  = 1.0;        // толщина платы XIAO (USB лежит сверху)
usb_xiao_z = z_pt + xiao_pcb + 1.63; // ось USB-C XIAO    23.83
usb_j1_z   = z_pt + 1.78;            // ось зарядного J1  22.98

// ---------- наружные габариты
X0 = ix0 - endw;  X1 = ix1 + endw;   // -37.65 … 36.45  (74.1 мм)
W  = 2 * (iy + wall);                // 27.9 мм
r_v = 2.0;  r_t = 1.5;               // скругления вертикальных и верхних рёбер

// ---------- седло и ремешки
sad_h  = 3.0;   sad_w = 16;          // высота и ширина седла
band_x = 31;    band_w = 10;  band_d = 0.4;   // канавки ремешков (x = ±31)

// ---------- органы управления (координаты платы)
btn    = [3.0, 7.4];      // SW2 TL3342, высота 1.54
btn_h  = 1.54;
led_r  = [11.0, 10.0];    // LED1 красный
led_g  = [11.0, 7.4];     // LED2 зелёный
led_c  = [24.8, -6.8];    // LED3 оранжевый — заряд
bz     = [20.4, 6.6];     // BZ1 CMT-8504, 8.5 x 8.5
sw1_x  = -9.5;            // SW1 PCM12 — ползунок на стороне -Y
ant_x  = -21.2;           // паз под кабель антенны XIAO (сторона +Y)

// ---------------------------------------------------------------------
module alongX(a, b) translate([a, 0, 0]) rotate([90, 0, 90]) linear_extrude(b - a) children();
module alongY(h)    rotate([90, 0, 0]) linear_extrude(h, center = true) children();
module box(x, y, z) translate([x[0], y[0], z[0]]) cube([x[1]-x[0], y[1]-y[0], z[1]-z[0]]);
module stadium(w, h) hull() for (s = [-1, 1]) translate([s * (w/2 - h/2), 0]) circle(d = h);

// профиль YZ корпуса: круглое дно по ячейке, прямые стенки, скруглённый верх
module prof_outer() hull() {
    translate([0, zc]) circle(r = cell_r + bot);
    translate([-W/2, 8]) square([W, z_top - r_t - 8]);
    for (s = [-1, 1]) translate([s * (W/2 - r_t), z_top - r_t]) circle(r = r_t);
}

module outer() intersection() {
    union() {
        alongX(X0, X1) prof_outer();
        saddle();
    }
    // скругление вертикальных рёбер
    translate([0, 0, -sad_h - 1]) linear_extrude(z_top + sad_h + 2)
        offset(r = r_v) offset(delta = -r_v) translate([X0, -W/2]) square([X1 - X0, W]);
    // скругление верхних поперечных рёбер
    alongY(W + 2) hull() {
        translate([X0, -sad_h - 1]) square([X1 - X0, z_top - r_t + sad_h + 1]);
        for (x = [X0 + r_t, X1 - r_t]) translate([x, z_top - r_t]) circle(r = r_t);
    }
}

module saddle() difference() {
    hull() {
        box([X0 + 1, X1 - 1], [-sad_w/2, sad_w/2], [-sad_h, 4]);
        box([X0, X1], [-sad_w/2 + 1, sad_w/2 - 1], [-sad_h + 1, 4]);
    }
    // V-канавка 90° под бланк удилища
    alongX(X0 - 1, X1 + 1) polygon([[-sad_h + 0.4, -sad_h - 0.01], [sad_h - 0.4, -sad_h - 0.01], [0, -0.4]]);
}

// внутренняя полость
module cavity() {
    alongX(ix0, ix1) translate([0, zc]) circle(r = cell_r);
    box([ix0, ix1], [-iy, iy], [zc, z_li]);
}

// рельсы-полки под края платы (с фаской 45° — печать без поддержек)
module rails() for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0])
    alongX(ix0, ix1) polygon([[iy + 0.01, 13.4], [iy + 0.01, z_pb], [iy - 1.05, z_pb], [iy - 1.05, 14.45]]);

// шип крышки, входящий в паз верхней кромки стенок нижней части
module tongue(cl = 0) for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0])
    box([ix0 + 0.1 - cl, ix1 - 0.1 + cl], [iy - cl, iy + wall/2 - 0.05 + cl], [z_s - tng - cl, z_s + 0.01]);

// защёлки: валик на шипе / канавка в стенке
snap_x = [-20, 20];
module snaps(grow = 0) for (s = [-1, 1], x = snap_x) mirror([0, s < 0 ? 1 : 0, 0])
    translate([x, iy + wall/2 - 0.05 - 0.2, z_s - 0.8]) rotate([0, 90, 0])
        cylinder(r = 0.5 + grow, h = 5 + 2 * grow, center = true, $fn = 24);

// отверстия USB-C в торцах + внешняя выемка под корпус штекера
module usb_cuts() {
    for (c = [[X0 - 1, ix0 + 0.5, usb_xiao_z, 4.0], [ix1 - 0.5, X1 + 1, usb_j1_z, 3.8]])
        alongX(c[0], c[1]) translate([0, c[2]]) stadium(9.4, c[3]);
    for (c = [[X0 - 1, X0 + 0.5, usb_xiao_z], [X1 - 0.5, X1 + 1, usb_j1_z]])
        alongX(c[0], c[1]) translate([0, c[2]]) offset(r = 1.5) square([12.6 - 3, 7.0 - 3], center = true);
}

// окно ползунка SW1 (сторона -Y) с выемкой под ноготь
module sw1_cut() {
    box([sw1_x - 2.9, sw1_x + 2.9], [-W/2 - 1, -iy + 0.5], [z_s - 0.01, z_pt + 1.9]);
    translate([sw1_x, -W/2 - 0.01, z_pt + 0.95]) rotate([-90, 0, 0])
        linear_extrude(1.1) offset(r = 1) square([10 - 2, 5 - 2], center = true);
}

// вертикальный паз под кабель антенны U.FL (сторона +Y)
module ant_cut() box([ant_x - 1, ant_x + 1], [iy - 1.1, iy + wall/2], [12, z_li - 0.5]);

// канавки ремешков на боковых стенках
module band_cuts() for (bx = [-band_x, band_x]) difference() {
    alongX(bx - band_w/2, bx + band_w/2) prof_outer();
    alongX(bx - band_w/2 - 1, bx + band_w/2 + 1) offset(delta = -band_d) prof_outer();
    box([X0 - 1, X1 + 1], [-W, W], [18, z_top + 1]);
    box([X0 - 1, X1 + 1], [-W, W], [-sad_h - 1, 4]);
}

// ---------------------------------------------------------------------
module tub() difference() {
    union() {
        difference() {
            intersection() { outer(); box([X0 - 1, X1 + 1], [-W, W], [-sad_h - 1, z_s]); }
            cavity();
        }
        intersection() { rails(); box([X0, X1], [-W, W], [0, z_pb]); }
    }
    tongue(cl = 0.05);
    snaps(grow = 0.1);
    usb_cuts(); sw1_cut(); ant_cut(); band_cuts();
}

// кнопка-язычок: петля у x = btn-9, толщина 0.8, толкатель на SW2
tab_x = [btn[0] - 9, btn[0] + 3.5];
tab_y = [btn[1] - 3, btn[1] + 3];
module lid() difference() {
    union() {
        difference() {
            intersection() { outer(); box([X0 - 1, X1 + 1], [-W, W], [z_s, z_top + 1]); }
            cavity();
        }
        tongue();
        snaps();
        // световоды LED1/LED2 (один блок) и LED3
        box([led_r[0] - 2, led_r[0] + 2], [led_g[1] - 1.4, led_r[1] + 1.6], [z_pt + 1.1, z_li + 0.01]);
        translate([led_c[0], led_c[1], z_pt + 1.1]) cylinder(d = 3.6, h = z_li - z_pt - 1.09);
        // прижимы платы по краям (свободные от деталей места)
        for (p = [[-12, 1], [13.5, 1], [30, 1], [-30, -1], [-4, -1], [15.5, -1], [27, -1]])
            box([p[0] - 0.8, p[0] + 0.8], p[1] > 0 ? [iy - 0.9, iy + 0.01] : [-iy - 0.01, -iy + 0.9],
                [z_pt + 0.05, z_li + 0.01]);
        // толкатель кнопки
        translate([btn[0], btn[1], z_pt + btn_h + 0.1]) cylinder(d = 2.2, h = z_li - z_pt - btn_h - 0.09);
    }
    // окна светодиодов
    for (l = [led_r, led_g, led_c]) translate([l[0], l[1], z_pt]) {
        cylinder(d = 1.8, h = 10);
        translate([0, 0, z_top - z_pt - 0.4]) cylinder(d1 = 1.8, d2 = 2.6, h = 0.41);
    }
    // зуммер: карман 0.8 мм над ним + звуковые отверстия
    box([bz[0] - 5.1, bz[0] + 5.1], [bz[1] - 5.1, min(bz[1] + 5.1, iy - 0.2)], [z_li - 0.01, z_li + 0.8]);
    for (d = [[0, 0], [2.4, 0], [-2.4, 0], [0, 2.4], [0, -2.4]])
        translate([bz[0] + d[0], bz[1] + d[1], z_li]) cylinder(d = 1.3, h = 5, $fn = 24);
    // карман под USB-C разъём XIAO (он выше экрана модуля)
    box([ix0 - 0.01, -28.5], [-5.5, 5.5], [z_li - 0.01, z_li + 0.7]);
    // язычок кнопки: U-образная прорезь 0.6 мм и утонение до 0.8 мм сверху
    difference() {
        box([tab_x[0], tab_x[1] + 0.6], [tab_y[0] - 0.6, tab_y[1] + 0.6], [z_li - 1, z_top + 1]);
        box([tab_x[0] - 1, tab_x[1]], [tab_y[0], tab_y[1]], [z_li - 2, z_top + 2]);
    }
    box([tab_x[0], tab_x[1]], [tab_y[0], tab_y[1]], [z_li + 0.8, z_top + 1]);
    usb_cuts(); sw1_cut(); ant_cut(); band_cuts();
}

// ---------------------------------------------------------------------
// макеты для проверки посадки
module pcba() {
    translate([0, 0, z_pb]) import("_fit/pcba.stl");
    // XIAO ESP32-S3 (нет 3D-модели в библиотеке) — модуль и его USB-C
    box([-35.2, -14.2], [-8.9, 8.9], [z_pt, z_pt + comp_h]);
    box([-36.33, -28.98], [-4.47, 4.47], [z_pt + xiao_pcb, z_pt + xiao_pcb + 3.26]);
    // зуммер CMT-8504 (модели нет) — берём 4 мм высоты с запасом
    translate([bz[0] - 4.25, bz[1] - 4.25, z_pt]) cube([8.5, 8.5, 4.0]);
}
cell_len = 69;   // защищённая ячейка; незащищённая 65
module cell() translate([(ix0 + ix1)/2 - cell_len/2, 0, bot + 9.25]) rotate([0, 90, 0]) cylinder(r = 9.25, h = cell_len);

if (part == "tub") tub();
else if (part == "lid") lid();
else if (part == "lid_print") translate([0, 0, z_top]) rotate([180, 0, 0]) lid();
else if (part == "assembly") {
    color("#2f6f8f") render() tub();
    color("#e8e4d8", 0.55) render() lid();
    color("#1b5e20") render() pcba();
    color("#c0392b") render() cell();
}
else if (part == "exploded") {
    color("#2f6f8f") render() tub();
    color("#e8e4d8") translate([0, 0, 22]) render() lid();
    color("#1b5e20") translate([0, 0, 11]) render() pcba();
    color("#c0392b") render() cell();
}
else if (part == "section") intersection() {
    union() { color("#2f6f8f") render() tub(); color("#e8e4d8") render() lid(); color("#1b5e20") render() pcba(); color("#c0392b") render() cell(); }
    box([X0 - 5, sec_x], [-W, W], [-10, 40]);
}
else if (part == "clash_lid") intersection() { lid(); pcba(); }
else if (part == "clash_tub") intersection() { tub(); union() { pcba(); cell(); } }
else if (part == "clash_lid_cell") intersection() { lid(); cell(); }
