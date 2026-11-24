#pragma once
#include <Arduino.h>
#include "SensorCore.h"

// Settings 1.4.3 custom-widget CSS: keep the installed library untouched.
// Label and value columns may wrap; native sliders/switches keep their controls.
const char benchMobileJs[] PROGMEM = R"JS(
class BenchLayout extends WidgetBase {
    constructor(data) { super(data, false); this.$root.style.display = 'none'; }
    static css = `
    .main .widget_row { height:auto; min-height:36px; gap:8px; }
    .main .widget_row > :first-child { flex:1 1 40%; min-width:0; }
    .main .widget_row > :last-child:not(:first-child) { min-width:0; max-width:58%; }
    .main .widget_row label { white-space:normal; overflow-wrap:anywhere; font-size:16px; line-height:1.35; margin-right:0; }
    .main .value { white-space:normal; overflow:visible; overflow-wrap:anywhere; text-overflow:clip; font-size:15px; text-align:right; }
    .main .value.active { flex-shrink:0; }
    .main .input_cont { width:auto; }
    .main .switch { flex-shrink:0; width:52px; }
    .main .button { white-space:normal; text-align:center; min-height:44px; font-size:16px; }
    .main .nav { min-width:0; font-size:17px; overflow-wrap:anywhere; }
    .main .header > :last-child { flex-shrink:0; }
    .main .group_col, .main .widget { min-width:0; }
    .main .slider_cont { padding-top:16px; }
    .main .paragraph { font-size:14px; line-height:1.45; color:var(--font_tint); overflow-wrap:anywhere; }
    `;
}
)JS";
