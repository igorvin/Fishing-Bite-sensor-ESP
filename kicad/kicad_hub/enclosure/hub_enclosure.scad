// Корпус хаба поклёвки rev A: «бипер» с откидной клипсой (ремень / подставка).
// Плата hub rev A 68 x 40 мм, OLED SH1106 1.3" над платой, LiPo 1200 мА·ч за платой.
// Все размеры в мм. Система координат: x вправо, y вверх (вид спереди), z от задней крышки к экрану.
// Точка (0,0) — левый нижний угол платы; координаты KiCad (u, v) переводятся как x = u - 100, y = 140 - v.
//
// part = "assembly" | "exploded" | "shell" | "cover" | "clip" | "pusher" | "flap" | "plug_top" | "gasket"
//        | "section_x" | "section_y" | "clash"
part = "assembly";
$fn = 40;

// ---------------------------------------------------------------- плата и детали на ней
PCB = [68, 40];  PCB_T = 1.0;  PCB_R = 2;
HOLES = [[2.2, 37.8], [2.2, 2.2], [43.3, 37.4], [41.2, 2.4]];      // H1..H4
OLED = [4.0, 4.1, 39.4, 37.6];            // x1 y1 x2 y2 модуля (вид спереди)
OLED_STANDOFF = 3.5;                       // зазор плата — плата модуля (гребёнка 2.5 + шайбы 1.0)
OLED_T = 3.1;                              // модуль: плата 1.6 + стекло 1.5
WIN = [31, 17];  WIN_C = [21.7, 20.6];     // окно под активную область 128x64 (сверить с модулем!)
PLATE = [34.6, 20.6, 1.0];                 // прозрачная пластина (акрил/поликарбонат 1 мм)
XIAO_C = [56, 28.4];                       // центр XIAO, USB-C у верхнего края платы
BTN = [52, 11.4];  BTN_H = 2.5;            // TL3342, высота над платой
LEDS = [[59.4, 11.4], [40.9, 14.2], [42.9, 14.2]];   // D5 (статус), D1 (заряд), D2 (USB есть)
USB_CHG_X = 50;                            // J1 USB-C зарядки, нижний край
SW_X = 61.5;                               // SW1 выключатель, нижний край
USB_XIAO_X = 56;                           // USB-C XIAO, верхний край (прошивка)

// ---------------------------------------------------------------- аккумулятор и мотор
BAT = [50, 34, 10];  BAT_POS = [2.0, 3.0]; // LiPo 1200 мА·ч, предположение 50 x 34 x 10 (уточнить!)
MOTOR = [61, 26];  MOTOR_D = 10.4;         // монетка 10 мм на задней крышке

// ---------------------------------------------------------------- корпус
WALL = 3.2;  GAP = 0.5;                    // стенка (толще у углов под винты) и зазор до платы
COVER_T = 2.5;                             // задняя крышка
Z_BAT = COVER_T;                           // низ аккумулятора
Z_PCB = COVER_T + BAT[2] + 2.0;            // низ платы: над аккумулятором 2 мм под головки винтов и выводы
Z_TOP = Z_PCB + PCB_T;                     // верх платы
Z_IN = Z_TOP + OLED_STANDOFF + OLED_T + 0.4;    // внутренняя плоскость передней стенки
FRONT_T = 1.8;
Z_OUT = Z_IN + FRONT_T;                    // наружная плоскость экрана
IN0 = [-GAP, -GAP];  IN1 = [PCB[0] + GAP, PCB[1] + GAP];
OUT0 = [IN0[0] - WALL, IN0[1] - WALL];  OUT1 = [IN1[0] + WALL, IN1[1] + WALL];
R_OUT = 3;
CORNERS = [[-1.5, -1.5], [PCB[0] + 1.5, -1.5], [-1.5, PCB[1] + 1.5], [PCB[0] + 1.5, PCB[1] + 1.5]];
SCREW_D = 1.8;  SCREW_CLR = 2.4;  BOSS_D = 4.6;

module rrect(p0, p1, r, h) {
    hull() for (x = [p0[0] + r, p1[0] - r], y = [p0[1] + r, p1[1] - r]) translate([x, y, 0]) cylinder(r = r, h = h);
}

module outer(h) { rrect(OUT0, OUT1, R_OUT, h); }
module inner(h) { rrect(IN0, IN1, 0.6, h); }

// ---------------------------------------------------------------- передняя оболочка
module shell() {
    difference() {
        union() {
            difference() {
                translate([0, 0, COVER_T]) outer(Z_OUT - COVER_T);
                translate([0, 0, COVER_T - 0.01]) inner(Z_IN - COVER_T + 0.01);
            }
            // угловые бобышки под винты крышки
            intersection() { translate([0, 0, COVER_T]) outer(Z_IN); for (c = CORNERS) translate([c[0], c[1], COVER_T]) cylinder(d = BOSS_D, h = Z_IN - COVER_T); }
            // бобышки, к которым снизу прикручена плата
            for (h = HOLES) translate([h[0], h[1], Z_TOP]) cylinder(d = 4.4, h = Z_IN - Z_TOP + 0.01);
            // световоды к светодиодам (тонкостенные трубки)
            for (l = LEDS) translate([l[0], l[1], Z_TOP + 1.2]) cylinder(d = 3.6, h = Z_IN - Z_TOP - 1.2 + 0.01);
            // направляющая толкателя кнопки
            translate([BTN[0], BTN[1], Z_TOP + BTN_H + 1.6]) cylinder(d = 7.2, h = Z_IN - Z_TOP - BTN_H - 1.6 + 0.01);
        }
        // винты крышки и платы
        for (c = CORNERS) translate([c[0], c[1], COVER_T - 0.1]) cylinder(d = SCREW_D, h = 12);
        for (h = HOLES) translate([h[0], h[1], Z_TOP - 0.1]) cylinder(d = SCREW_D, h = 5);
        // окно экрана: сквозное окно + карман под прозрачную пластину изнутри
        translate([WIN_C[0], WIN_C[1], Z_IN - 0.01]) rrect([-WIN[0] / 2, -WIN[1] / 2], [WIN[0] / 2, WIN[1] / 2], 1, FRONT_T + 1);
        translate([WIN_C[0], WIN_C[1], Z_IN - 0.01]) rrect([-PLATE[0] / 2 - 0.2, -PLATE[1] / 2 - 0.2], [PLATE[0] / 2 + 0.2, PLATE[1] / 2 + 0.2], 0.8, PLATE[2] + 0.01);
        // кнопка: отверстие толкателя Ø4.1 (O-кольцо 2.5 x 1.0 на толкателе)
        translate([BTN[0], BTN[1], Z_TOP]) cylinder(d = 4.1, h = 20);
        translate([BTN[0], BTN[1], Z_TOP]) cylinder(d = 5.4, h = Z_IN - Z_TOP - 0.01);   // ход фланца
        // светодиоды: глухие окна, стенка 0.6 мм (печатать полупрозрачным PETG)
        for (l = LEDS) translate([l[0], l[1], Z_TOP]) cylinder(d = 2.4, h = Z_IN - Z_TOP + FRONT_T - 0.6);
        // нижний торец: USB-C зарядки и выключатель под общей ТПУ-заглушкой
        translate([USB_CHG_X, IN0[1] + 0.5, Z_TOP + 1.6]) rotate([90, 0, 0]) rrect([-4.8, -1.9], [4.8, 1.9], 1.2, WALL + 1);
        translate([SW_X, IN0[1] + 0.5, Z_TOP + 0.9]) rotate([90, 0, 0]) rrect([-2.6, -1.1], [2.6, 1.1], 0.5, WALL + 1);
        translate([(USB_CHG_X + SW_X) / 2, OUT0[1] - 0.01, Z_TOP + 1.3]) rotate([-90, 0, 0])
            rrect([-11.5, -4.2], [11.5, 4.2], 1.5, 2.0);                                   // гнездо заглушки: стенка у USB 1.2 мм
        translate([(USB_CHG_X + SW_X) / 2 + 13.5, OUT0[1] - 0.01, Z_TOP + 1.3]) rotate([-90, 0, 0]) cylinder(d = 2.2, h = 3.0);  // якорь ремешка
        // верхний торец: USB-C XIAO для прошивки, ТПУ-пробка
        translate([USB_XIAO_X, IN1[1] - 0.5, Z_TOP + 1.0 + 1.6]) rotate([-90, 0, 0]) rrect([-5.0, -2.0], [5.0, 2.0], 1.0, WALL + 1);
        translate([USB_XIAO_X, OUT1[1] + 0.01, Z_TOP + 1.0 + 1.6]) rotate([90, 0, 0]) rrect([-6.0, -3.0], [6.0, 3.0], 1.5, 2.0);   // гнездо пробки
        // место под модуль OLED (бобышки H1/H2 и световод D1 подрезаются)
        translate([OLED[0] - 0.3, OLED[1] - 0.3, Z_TOP + OLED_STANDOFF - 0.3]) cube([OLED[2] - OLED[0] + 0.6, OLED[3] - OLED[1] + 0.6, OLED_T + 0.6]);
        // паз под плоскую прокладку по торцу
        translate([0, 0, COVER_T - 0.01]) difference() { outer(0.6); inner(0.6); }
    }
}

// ---------------------------------------------------------------- задняя крышка
module cover() {
    difference() {
        union() {
            outer(COVER_T);
            // рамка аккумулятора
            translate([0, 0, COVER_T - 0.01]) difference() {
                rrect([BAT_POS[0] - 1.2, BAT_POS[1] - 1.2], [BAT_POS[0] + BAT[0] + 1.2, BAT_POS[1] + BAT[1] + 1.2], 1.5, 2);
                rrect([BAT_POS[0] - 0.2, BAT_POS[1] - 0.2], [BAT_POS[0] + BAT[0] + 0.2, BAT_POS[1] + BAT[1] + 0.2], 1, 3);
                translate([BAT_POS[0] + BAT[0] - 2, BAT_POS[1] + 6, -1]) cube([4, BAT[1] - 12, 4]);   // выход проводов
            }
            // кольцо мотора
            translate([MOTOR[0], MOTOR[1], COVER_T - 0.01]) difference() { cylinder(d = MOTOR_D + 2, h = 1.6); translate([0, 0, -1]) cylinder(d = MOTOR_D, h = 4); }
            // петли клипсы снаружи (сверху)
            for (x = HINGE_X) translate([x, 0, 0]) knuckle_cover();
        }
        for (c = CORNERS) translate([c[0], c[1], -0.1]) { cylinder(d = SCREW_CLR, h = 5); cylinder(d = 3.6, h = 1.4); }
        translate([MOTOR[0] - 2, MOTOR[1] + MOTOR_D / 2 - 0.5, COVER_T]) cube([4, 3, 3]);   // провода мотора
    }
}

// ---------------------------------------------------------------- клипса: на ремень (сложена) / подставка (откинута на ~42°)
CLIP_W = 34;  CLIP_L = 40;  CLIP_T = 2.6;  AXIS_Y = PCB[1] - 3;  AXIS_Z = -2.6;  BELT = 3.2;
HINGE_X = [PCB[0] / 2 - CLIP_W / 2 - 0.2 - 5, PCB[0] / 2 + CLIP_W / 2 + 0.2];   // две петли по 5 мм
module knuckle_cover() {
    difference() {
        hull() {
            translate([0, AXIS_Y - 4, 0]) cube([5, 8, 0.01]);
            translate([0, AXIS_Y, AXIS_Z]) rotate([0, 90, 0]) cylinder(r = 2.4, h = 5);
        }
        translate([-1, AXIS_Y, AXIS_Z]) rotate([0, 90, 0]) cylinder(d = SCREW_CLR, h = 7);
    }
}
module clip() {   // в сложенном положении, ось вдоль x
    x0 = PCB[0] / 2 - CLIP_W / 2;
    difference() {
        union() {
            translate([x0, AXIS_Y, AXIS_Z]) rotate([0, 90, 0]) cylinder(r = 2.4, h = CLIP_W);
            // упор: при откидывании на ~42° выступ ложится на крышку
            translate([x0 + 6, AXIS_Y, AXIS_Z - 1.2]) cube([CLIP_W - 12, 3.4, 2.4]);
            // пластина клипсы: зазор BELT до крышки под ремень
            translate([x0, AXIS_Y - CLIP_L, AXIS_Z - BELT - CLIP_T + 2.4]) cube([CLIP_W, CLIP_L, CLIP_T]);
            translate([x0, AXIS_Y - 3, AXIS_Z - BELT - CLIP_T + 2.4]) cube([CLIP_W, 3, BELT + CLIP_T - 2.4]);
            // зацеп на конце: прижимает ремень
            translate([x0, AXIS_Y - CLIP_L, AXIS_Z - BELT + 2.4 - 0.01]) cube([CLIP_W, 2.2, 1.6]);
        }
        translate([x0 - 1, AXIS_Y, AXIS_Z]) rotate([0, 90, 0]) cylinder(d = SCREW_CLR, h = CLIP_W + 2);
        for (x = [x0 - 0.01, x0 + CLIP_W - 0.0]) ;
    }
}

// ---------------------------------------------------------------- мелкие детали
module pusher() {    // толкатель кнопки: шток Ø4.0, канавка под O-кольцо 2.5 x 1.0, фланец внутри
    L = Z_OUT - (Z_TOP + BTN_H) + 0.8;     // выступает на 0.8 мм над корпусом
    difference() {
        union() { cylinder(d = 4.0, h = L); cylinder(d = 5.0, h = 1.2); }
        translate([0, 0, L - 2.6]) difference() { cylinder(d = 5, h = 1.0); cylinder(d = 2.6, h = 1.0); }
    }
}
module flap() {      // ТПУ-заглушка нижнего торца: пробка USB-C + клапан над выключателем + ремешок
    rrect([-11.3, -4.0], [11.3, 4.0], 1.4, 1.8);
    translate([USB_CHG_X - (USB_CHG_X + SW_X) / 2, 0.3, 1.79]) rrect([-4.5, -1.6], [4.5, 1.6], 1.0, 3.0);
    translate([11.3, -1.0, 0]) cube([2.0, 2.0, 0.8]);
    translate([13.5, 0, 0]) { cylinder(d = 2.2, h = 3.2); translate([0, 0, 3.2]) cylinder(d1 = 3.0, d2 = 2.2, h = 1.0); }
}
module plug_top() {  // ТПУ-пробка USB-C XIAO
    rrect([-5.8, -2.8], [5.8, 2.8], 1.4, 1.8);
    translate([0, 0, 1.79]) rrect([-4.5, -1.6], [4.5, 1.6], 1.0, 3.0);
}
module gasket() {    // плоская прокладка 0.8 мм (ТПУ 85A) по торцу оболочки
    difference() { outer(0.8); inner(0.8); for (c = CORNERS) translate([c[0], c[1], -1]) cylinder(d = SCREW_CLR, h = 3); }
}

// ---------------------------------------------------------------- макеты для сборки и проверок
module pcba() {
    color("darkgreen") translate([0, 0, Z_PCB]) rrect([0, 0], PCB, PCB_R, PCB_T);
    color("dimgray") translate([OLED[0], OLED[1], Z_TOP + OLED_STANDOFF]) cube([OLED[2] - OLED[0], OLED[3] - OLED[1], 1.6]);
    color("black") translate([OLED[0] + 0.5, WIN_C[1] - 12, Z_TOP + OLED_STANDOFF + 1.6]) cube([OLED[2] - OLED[0] - 1, 24, 1.5]);
    color("royalblue") translate([XIAO_C[0] - 8.75, XIAO_C[1] - 10.5, Z_TOP]) cube([17.5, 21, 1.0]);
    color("silver") translate([XIAO_C[0] - 4.5, PCB[1] - 7.5 + 1.2, Z_TOP + 1.0]) cube([9, 7.5, 3.2]);
    color("silver") translate([USB_CHG_X - 4.5, -0.5, Z_TOP]) cube([9, 7.3, 3.2]);
    color("white") translate([SW_X - 4.5, 0.4, Z_TOP]) cube([9, 3.6, 1.4]);
    color("white") translate([SW_X - 1, -0.9, Z_TOP + 0.3]) cube([2, 1.4, 1.2]);
    color("black") translate([BTN[0] - 2.25, BTN[1] - 1.75, Z_TOP]) cube([4.5, 3.5, BTN_H]);
    color("ivory") translate([56, 30, Z_PCB - 4.8]) cube([8, 6, 4.8]);    // JST-PH снизу
    color("lightblue") translate([BAT_POS[0], BAT_POS[1], Z_BAT]) cube(BAT);
    color("gray") translate([MOTOR[0], MOTOR[1], COVER_T]) cylinder(d = 10, h = 3);
}

module assembly(e = 0) {
    color("orange", 0.9) translate([0, 0, 2 * e]) shell();
    color("tan") cover();
    color("peru") clip();
    color("crimson") translate([BTN[0], BTN[1], Z_TOP + BTN_H + 0.2 + 2.5 * e]) pusher();
    translate([0, 0, e]) pcba();
}

if (part == "assembly") assembly();
if (part == "exploded") assembly(12);
if (part == "shell") translate([0, 0, Z_OUT]) rotate([180, 0, 0]) shell();     // печать экраном вниз
if (part == "cover") cover();                                                    // наружной гранью на стол
if (part == "clip") rotate([0, -90, 0]) clip();
if (part == "pusher") pusher();
if (part == "flap") flap();
if (part == "plug_top") plug_top();
if (part == "gasket") gasket();
if (part == "section_x") difference() { assembly(); translate([BTN[0], -50, -50]) cube(200); }
if (part == "back") assembly();
if (part == "section_y") difference() { assembly(); translate([-50, WIN_C[1], -50]) cube(200); }
// пересечения оболочки с макетом платы и деталями: должен получиться пустой объект
if (part == "clash") intersection() { shell(); pcba(); }
