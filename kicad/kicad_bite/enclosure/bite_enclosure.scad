// =====================================================================
//  BITE SENSOR v4 — герметичный корпус-пенал (enclosure v2)
//  Плата 70 x 24 x 1.0 мм (kicad_bite/bite_sensor.kicad_pcb, v4) сверху,
//  ячейка 18650 снизу. Плата с ячейкой вдвигаются в цельный корпус с левого
//  торца; торец закрывается крышкой с радиальным O-кольцом и 2 винтами M2.
//  Магнитный разъём зарядки вклеен в крышку. Кнопка — толкатель с O-кольцом,
//  окна светодиодов — глухие (стенка 0.6 мм), зуммер — за мембраной ePTFE.
//
//  Система координат: начало = центр платы (KiCad 135, 112),
//  X — вдоль корпуса (XIAO и крышка слева, кнопка справа),
//  Y — вверх на виде сверху (= -Y KiCad), Z = 0 — низ корпуса (без седла).
//
//  part = "body" | "cap" | "pusher" | "retainer"         — детали
//         "body_print" | "cap_print" | "pusher_print" | "retainer_print"
//         "assembly" | "exploded" | "section" (X = sec_x) | "section_long" (Y = sec_y)
//         "clash_*"  — проверки пересечений (результат должен быть пустым)
// =====================================================================
part  = "assembly";
sec_x = 31.0;     // поперечный разрез (через кнопку)
sec_y = 0.0;      // продольный разрез
$fn = 72;

// ---------- плата
pcb_t = 1.0;
bx0   = -38.0;             // левый край платы v4.1 (плата 73 мм: -38…35)
ix0   = -39.45;            // внутренний торец слева (нос USB-C XIAO -39.33)
ix1   = 35.25;             // глухой торец справа (край платы 35.0)
iy    = 12.35;             // внутренняя полуширина (край платы 12.0)

// ---------- стенки
wall  = 2.0;               // все стенки (5 периметров 0.4) — меньше для FDM нельзя
skin  = 0.6;               // стенка над светодиодами (пропускает свет)

// ---------- ячейка 18650
cell_r = 9.45;             // ложемент 18.9 мм (ячейка 18.3…18.6 + лента)
zc     = wall + cell_r;            // ось ложемента   11.45
z_pb   = zc + cell_r + 0.1;        // низ платы       21.0
z_pt   = z_pb + pcb_t;             // верх платы      22.0
gap_t  = 4.6;                      // над платой: USB XIAO 4.26, зуммер 4.0
z_ci   = z_pt + gap_t;             // потолок         26.6
z_top  = z_ci + wall;              // верх корпуса    28.6
r_in   = 2.5;                      // скругление внутренних углов профиля

// ---------- длина
plug_l = 5.0;                      // глубина пробки крышки
X0     = ix0 - plug_l;             // открытый торец корпуса  -44.45
X1     = ix1 + wall;               // глухой торец             37.25
cap_t  = 2.0;                      // фланец крышки
XC     = X0 - cap_t;               // наружная грань крышки   -46.45
W      = 2 * (iy + wall);          // 28.7

// ---------- уплотнение крышки (радиальное, шнур/кольцо сечением 1.5 мм)
clr    = 0.10;                     // зазор пробка — отверстие (на сторону)
or_cs  = 1.5;                      // сечение O-кольца
gr_d   = 1.05;                     // глубина канавки -> сжатие (1.5-1.15)/1.5 = 23 %
gr_w   = 2.0;                      // ширина канавки
gr_x   = X0 + 1.8;                 // канавка: X0+1.8 … X0+3.8
plug_w = 2.6;                      // стенка пробки

// ---------- седло и ремешки
sad_h = 3.0;  sad_w = 16;
band_x = [-30.5, 7.5];  band_w = 10;  band_d = 0.4;

// ---------- органы управления (координаты платы v4)
btn   = [31.0, 0.0];   btn_h = 1.54;      // SW2 TL3342
leds  = [[31.0, 6.5], [31.0, -6.5], [24.8, -6.8]];   // LED1 красн., LED2 зел., LED3 заряд
bz    = [20.4, 6.6];                      // BZ1 CMT-8504, 8.5 x 8.5 x 4
chg   = [-36.6, 10.45];                   // площадки J1 (+ сверху, - снизу)
jst   = [-37.6, -32.5];                   // J2 JST-PH 4P снизу платы (корпус разъёма по x)

// ---------- толкатель кнопки
boss_d = 7.0;  boss_h = 2.0;              // бобышка на крышке корпуса
sh_d   = 4.0;  bore_d = 4.1;              // шток / отверстие (развернуть сверлом 4.1)
po_gd  = 2.5;  po_gw = 1.3;               // канавка штока под O-кольцо 2.5 x 1.0
pg     = 0.55;                            // зазор головка — бобышка (= ход)
hd_d   = 6.4;  hd_t = 0.6;                // фланец головки
nub_d  = 4.4;  nub_h = 1.2;               // выступ, на который нажимают
tip_gap = 0.3;                            // зазор шток — SW2 в покое

// ---------- магнитный разъём (гнездо с плоскими контактами, вклеивается в крышку)
mag_w = 8.6;  mag_h = 3.6;  mag_d = 3.0;  // ИЗМЕРИТЬ свой разъём и поправить
mag_p = 2.5;  mag_z = zc;                 // шаг контактов, высота центра

// ---------- винты крышки: M2 + латунные вставки 3.2 x 3 в торце седла
scr = [[-4.6, 0.0], [4.6, 0.0]];

// ---------------------------------------------------------------------
module alongX(a, b) translate([a, 0, 0]) rotate([90, 0, 90]) linear_extrude(b - a) children();
module alongY(h)    rotate([90, 0, 0]) linear_extrude(h, center = true) children();
module box(x, y, z) translate([x[0], y[0], z[0]]) cube([x[1]-x[0], y[1]-y[0], z[1]-z[0]]);
module toYZ() rotate([-90, 0, 0]) rotate([0, 0, -90]) children();   // обратное alongX

// профиль полости (YZ): выпуклый — ложемент + прямоугольник над ним
module bore2d(d = 0) offset(delta = d) offset(r = r_in) offset(delta = -r_in) hull() {
    translate([0, zc]) circle(r = cell_r);
    translate([-iy, zc]) square([2 * iy, z_ci - zc]);
}
module outer2d() offset(r = wall) bore2d();

module saddle2d() difference() {
    translate([-sad_w/2, -sad_h]) square([sad_w, sad_h + 4]);
    polygon([[-sad_h + 0.4, -sad_h - 0.01], [sad_h - 0.4, -sad_h - 0.01], [0, -0.4]]);
}
module saddle() difference() {
    hull() {
        box([X0, X1 - 1], [-sad_w/2, sad_w/2], [-sad_h, 4]);            // левый торец — полный (вставки)
        box([X0, X1], [-sad_w/2 + 1, sad_w/2 - 1], [-sad_h + 1, 4]);
    }
    alongX(X0 - 1, X1 + 1) polygon([[-sad_h + 0.4, -sad_h - 0.01], [sad_h - 0.4, -sad_h - 0.01], [0, -0.4]]);
}

// полки (снизу) и упоры (сверху) по краям платы
module rails() for (s = [-1, 1]) mirror([0, s < 0 ? 1 : 0, 0]) alongX(ix0 + 0.6, ix1) {
    polygon([[iy + 0.01, z_pb - 2], [iy + 0.01, z_pb], [iy - 1.0, z_pb], [iy - 1.0, z_pb - 1]]);
    polygon([[iy + 0.01, z_pt + 0.3], [iy - 0.6, z_pt + 0.3], [iy - 0.6, z_pt + 0.9], [iy + 0.01, z_pt + 1.5]]);
}

module band_cuts() for (bx = band_x) difference() {
    alongX(bx - band_w/2, bx + band_w/2) outer2d();
    alongX(bx - band_w/2 - 1, bx + band_w/2 + 1) offset(delta = -band_d) outer2d();
    box([X0 - 1, X1 + 1], [-W, W], [19, z_top + 1]);
    box([X0 - 1, X1 + 1], [-W, W], [-sad_h - 1, 4]);
}

// ушко для страховочного шнура (Ø2.4) на правом торце — корпус тонет
module lanyard(cut = false) {
    ly = [5.5, 13.0];  lz = z_top + 2.4;  lx = [X1 - 2.0, X1];
    if (!cut) hull() {
        box(lx, ly, [z_top - 4, z_top]);
        translate([lx[0], (ly[0] + ly[1]) / 2, lz]) rotate([0, 90, 0]) cylinder(d = ly[1] - ly[0], h = lx[1] - lx[0]);
    }
    else translate([lx[0] - 1, (ly[0] + ly[1]) / 2, lz]) rotate([0, 90, 0]) cylinder(d = 2.4, h = 5, $fn = 32);
}

// ---------------------------------------------------------------------
module body() difference() {
    union() {
        difference() {
            union() { alongX(X0, X1) outer2d(); saddle(); lanyard(); }
            alongX(X0 - 1, ix1) bore2d();
        }
        rails();
        // бобышка толкателя
        translate([btn[0], btn[1], z_top - 0.5]) cylinder(d = boss_d, h = boss_h + 0.5);
    }
    // заходная фаска 1 мм под O-кольцо
    hull() { alongX(X0 - 0.01, X0) bore2d(0.6); alongX(X0 + 1.0, X0 + 1.01) bore2d(); }
    // отверстие толкателя
    translate([btn[0], btn[1], z_ci - 1]) cylinder(d = bore_d, h = 10, $fn = 48);
    // глухие окна светодиодов: снаружи Ø3, остаётся skin
    for (l = leds) translate([l[0], l[1], z_ci + skin]) cylinder(d = 3.0, h = 5);
    // зуммер: 5 отверстий + карман Ø8.4 x 0.5 изнутри под мембрану ePTFE Ø8
    for (d = [[0, 0], [2.2, 0], [-2.2, 0], [0, 2.2], [0, -2.2]])
        translate([bz[0] + d[0], bz[1] + d[1], z_ci - 1]) cylinder(d = 1.3, h = 5, $fn = 24);
    translate([bz[0], bz[1], z_ci - 0.5]) cylinder(d = 8.4, h = 1.0);
    // гнёзда латунных вставок M2 в торце седла
    for (s = scr) translate([X0 - 0.01, s[0], s[1]]) rotate([0, 90, 0]) cylinder(d = 3.2, h = 4.5, $fn = 32);
    band_cuts();
    lanyard(cut = true);
}

module cap() difference() {
    union() {
        // фланец: наружный профиль корпуса + седло, фаска 0.6 по краю
        hull() {
            alongX(XC, XC + 0.6) offset(delta = -0.6) union() { outer2d(); saddle2d(); }
            alongX(XC + 0.6, X0) union() { outer2d(); saddle2d(); }
        }
        // пробка
        difference() {
            hull() { alongX(X0 - 0.01, ix0 - 0.5) bore2d(-clr); alongX(ix0 - 0.5, ix0) bore2d(-clr - 0.5); }
            alongX(X0 - 0.02, ix0 + 0.1) bore2d(-clr - plug_w);
        }
        // упоры платы (давят на торец платы под модулем XIAO)
        for (s = [-1, 1]) box([ix0 - 0.5, bx0 - 0.3], s > 0 ? [5.6, 7.4] : [-7.4, -5.6], [z_pb, z_ci - clr - plug_w + 0.5]);
        // бобышка разъёма
        alongX(XC + 1, XC + mag_d + 0.9) translate([0, mag_z]) square([mag_w + 2.4, mag_h + 2.4], center = true);
    }
    // канавка O-кольца
    alongX(gr_x, gr_x + gr_w) difference() { bore2d(1); bore2d(-clr - gr_d); }
    saddle_groove();
    // гнездо магнитного разъёма (вклеить на эпоксидку) + отверстия под выводы
    alongX(XC - 0.01, XC + mag_d + 0.1) translate([0, mag_z]) square([mag_w + 0.2, mag_h + 0.2], center = true);
    for (s = [-1, 1]) translate([XC + 1, s * mag_p / 2, mag_z]) rotate([0, 90, 0]) cylinder(d = 1.3, h = 10, $fn = 24);
    // метка «+» (верхний контакт = VBUS, магниты разъёма не дают перевернуть)
    translate([XC - 0.01, 0, mag_z + mag_h/2 + 2.6]) rotate([90, 0, 90]) linear_extrude(0.4)
        text("+", size = 3, halign = "center", valign = "center");
    // винты M2
    for (s = scr) translate([XC - 1, s[0], s[1]]) rotate([0, 90, 0]) cylinder(d = 2.4, h = 10, $fn = 32);
    // паз для отвёртки, чтобы снять крышку
    translate([X0 - 0.6, 0, z_top - 0.6]) cube([1.4, 6, 1.6], center = true);
}
module saddle_groove() alongX(XC - 1, X0 + 0.01) polygon([[-sad_h + 0.4, -sad_h - 0.01], [sad_h - 0.4, -sad_h - 0.01], [0, -0.4]]);

// толкатель: головка снаружи, шток с O-кольцом 2.5 x 1.0 в отверстии
z_tip  = z_pt + btn_h + tip_gap;                 // низ штока в покое   23.84
z_head = z_top + boss_h + pg;                    // низ головки в покое 31.2
module pusher() difference() {
    union() {
        translate([btn[0], btn[1], z_tip]) cylinder(d = sh_d, h = z_head - z_tip + 0.01);
        translate([btn[0], btn[1], z_head]) cylinder(d = hd_d, h = hd_t);
        translate([btn[0], btn[1], z_head]) cylinder(d = nub_d, h = hd_t + 0.8 + nub_h);
    }
    // канавка в середине отверстия (и в покое, и при нажатии — внутри отверстия)
    translate([btn[0], btn[1], (z_ci + z_top + boss_h) / 2 + pg / 2 - po_gw / 2]) difference() {
        cylinder(d = sh_d + 1, h = po_gw);
        translate([0, 0, -1]) cylinder(d = po_gd, h = po_gw + 2);
    }
}
// фиксатор: надевается на бобышку и приклеивается, держит головку
module retainer() translate([btn[0], btn[1], z_top + 0.01]) difference() {
    cylinder(d = boss_d + 1.8, h = boss_h + pg + hd_t + 0.8);
    translate([0, 0, -1]) cylinder(d = boss_d + 0.2, h = boss_h + 1);
    translate([0, 0, boss_h - 0.01]) cylinder(d = hd_d + 0.3, h = pg + hd_t + 0.01);
    cylinder(d = nub_d + 0.4, h = 20);
}

// ---------------------------------------------------------------------
// макеты для проверки посадки
module pcba() {
    translate([0, 0, z_pb]) import("_fit/pcba.stl");
    box([-38.2, -17.2], [-8.9, 8.9], [z_pt, z_pt + 3.5]);                        // XIAO ESP32-S3
    box([-39.33, -31.98], [-4.47, 4.47], [z_pt + 1.0, z_pt + 1.0 + 3.26]);        // его USB-C
    box(jst, [-6.1, 6.1], [z_pb - 6.0, z_pb]);                                   // J2 B4B-PH-SM4-TB (снизу)
    box([jst[0] + 0.3, jst[1] - 0.3], [-5.0, 5.0], [z_pb - 9.6, z_pb - 6.0]);    // вставленная вилка PHR-4
    box([jst[0] + 0.8, jst[1] - 0.8], [-4.0, 4.0], [wall + 1.0, z_pb - 9.6]);    // изгиб проводов к ячейке
    translate([bz[0] - 4.25, bz[1] - 4.25, z_pt]) cube([8.5, 8.5, 4.0]);         // зуммер
    for (s = [-1, 1]) translate([chg[0], s * chg[1], z_pb - 1.5]) cylinder(d = 1.2, h = 1.5 + pcb_t + 1.5); // провода J1
}
cell_len = 65;     // ячейка без платы защиты (защита на плате)
cell_x0  = jst[1] + 0.6;   // ячейка начинается за разъёмом J2: -31.9 … 33.1, у правого торца пена 1.5–2 мм
module cell() translate([cell_x0, 0, wall + 9.25]) rotate([0, 90, 0]) cylinder(r = 9.25, h = cell_len);
module magcon() alongX(XC + 0.05, XC + mag_d) translate([0, mag_z]) square([mag_w, mag_h], center = true);
// объём, который заметает плата с ячейкой при вдвигании (проекция на YZ, протянутая вдоль X)
module sweep() alongX(X0 - 2, ix1 - 0.3) projection() toYZ() union() { pcba(); cell(); }

module cut() if (part == "section") box([XC - 5, sec_x], [-W, W], [-10, 45]);
             else box([XC - 5, X1 + 5], [-W, sec_y], [-10, 45]);
module show(c, a = 1) color(c, a) render() children();
if (part == "body") body();
else if (part == "cap") cap();
else if (part == "pusher") pusher();
else if (part == "retainer") retainer();
else if (part == "body_print") translate([0, 0, X1]) rotate([0, 90, 0]) body();
else if (part == "cap_print") translate([0, 0, -XC]) rotate([0, -90, 0]) cap();
else if (part == "pusher_print") translate([0, 0, z_head + hd_t + 0.8 + nub_h]) rotate([180, 0, 0]) translate([-btn[0], -btn[1], 0]) pusher();
else if (part == "retainer_print") translate([0, 0, z_top + boss_h + pg + hd_t + 0.8]) rotate([180, 0, 0]) translate([-btn[0], -btn[1], 0]) retainer();
else if (part == "assembly") {
    show("#2f6f8f") body();  show("#e8e4d8") cap();
    show("#f2b134") pusher(); show("#444") retainer();
    show("#1b5e20") pcba();  show("#c0392b") cell();  show("#b0b0b0") magcon();
}
else if (part == "exploded") {
    show("#2f6f8f") body();
    show("#e8e4d8") translate([-22, 0, 0]) cap();
    show("#b0b0b0") translate([-22, 0, 0]) magcon();
    show("#1b5e20") translate([-105, 0, 10]) pcba();
    show("#c0392b") translate([-105, 0, 0]) cell();
    show("#f2b134") translate([0, 0, 14]) pusher();
    show("#444") translate([0, 0, 24]) retainer();
}
else if (part == "section" || part == "section_long") {
    color("#2f6f8f") render() intersection() { body(); cut(); }
    color("#e8e4d8") render() intersection() { cap(); cut(); }
    color("#f2b134") render() intersection() { pusher(); cut(); }
    color("#444")    render() intersection() { retainer(); cut(); }
    color("#1b5e20") render() intersection() { pcba(); cut(); }
    color("#c0392b") render() intersection() { cell(); cut(); }
    color("#b0b0b0") render() intersection() { magcon(); cut(); }
}
else if (part == "clash_body")   intersection() { body(); union() { pcba(); cell(); } }
else if (part == "clash_sweep")  intersection() { body(); sweep(); }
else if (part == "clash_cap")    intersection() { cap(); union() { pcba(); cell(); body(); } }
else if (part == "clash_pusher") intersection() { pusher(); union() { pcba(); body(); } }
// плата выезжает, не задевая толкатель: всё, что правее x = 28, протягиваем влево
else if (part == "clash_slide_pusher") intersection() {
    pusher();
    alongX(X0 - 2, ix1) projection() toYZ() intersection() { pcba(); box([28, 40], [-W, W], [0, 40]); }
}
else if (part == "clash_ret")    intersection() { retainer(); union() { body(); pusher(); } }
