var label = func(txt, posx, posy, desc) {
  cDefaultGroup.createChild("text", desc)
                .setTranslation(posx, posy)
                .setAlignment("left-top")
                .setFont("typewriter.txf")
                .setFontSize(40, 1.5)
                .setColor(0,0,0)
                .setText(txt);
};

var variable = func(txt, posy, desc) {
  cDefaultGroup.createChild("text", desc)
                .setTranslation(665, posy)
                .setAlignment("right-top")
                .setFont("DSEG/DSEG7/Classic/DSEG7Classic-BoldItalic.ttf")
                .setFontSize(80, 1.2)
                .setColor(0,0,0)
                .setText(txt);
};

var image = func(path, posx, posy, sizex, sizey) {
  cDefaultGroup.createChild("image")
               .setFile(path)
               .setTranslation(posx, posy)
               .setSize(sizex, sizey);
}

var needle = func() {
  cDefaultGroup.createChild("path")
               .setColor(0,0,0);
}

# hash table for needle positions
var needles = [[0,0,0,0,0,0,0,0,0,0]];
for (var i=0; i<26; i = i+1) {
    var r0 = 107; var r1 = 268;
    var a0 = (i*4.32 - 1.5) * 0.0174533;
    var a1 = (i*4.32 + 1.5) * 0.0174533;

    var r3 = r1+8;
    var a3 = (a0 + a1)/2;

    var x1 = r0 * math.cos(a0);
    var y1 = r0 * math.sin(a0);

    var x2 = r1 * math.cos(a0);
    var y2 = r1 * math.sin(a0);

    var x3 = r3 * math.cos(a3);
    var y3 = r3 * math.sin(a3);

    var x4 = r1 * math.cos(a1);
    var y4 = r1 * math.sin(a1);

    var x5 = r0 * math.cos(a1);
    var y5 = r0 * math.sin(a1);

    append(needles, [x1, y1, x2, y2, x3, y3, x4, y4, x5, y5]);
}

var draw_vario = func(needle_t, vario) {
  var min = 0; var max = 0; var sign = 1;

  if (vario < -5) {
    if (vario < -9.9) vario = -9.9;
    min = math.round(-5*vario-25);
    max = 26;
    sign = -1;
  }
  else if (vario < 0) {
    min = 0;
    max = 1+math.round(-5*vario);
    sign = -1;
  }
  else if (vario > 5) {
    if (vario > 9.9) vario = 9.9;
    min = math.round(5*vario-25);
    max = 26;
  }
  else {
    min = 0;
    max = 1+math.round(5*vario);
  }

  needle_t.reset();
  var cx = 349; var cy = 361;
  for (var i=min+1; i<max+1; i = i+1) {
    var x = cx - needles[i][0];
    var y = cy - sign*needles[i][1];
    needle_t.moveTo(x, y);

    var x = cx - needles[i][2];
    var y = cy - sign*needles[i][3];
    needle_t.lineTo(x, y);

    var x = cx - needles[i][4];
    var y = cy - sign*needles[i][5];
    needle_t.lineTo(x, y);

    var x = cx - needles[i][6];
    var y = cy - sign*needles[i][7];
    needle_t.lineTo(x, y);

    var x = cx - needles[i][8];
    var y = cy - sign*needles[i][9];
    needle_t.lineTo(x, y);
    needle_t.close();
  }
  needle_t.setColorFill(0,0,0);
}

var cDisplay = canvas.new({
  "name": "IQONE",
  "size": [1024, 1024],
  "view": [795, 1024],
  "mipmapping": 1
});
cDisplay.addPlacement({"node": "iq_display"});
cDisplay.set("background", canvas.style.getColor("bg_color"));

var window = canvas.Window.new([240,309],"dialog");
window.setCanvas(cDisplay);

var cDefaultGroup = cDisplay.createGroup();

label( "m/s", 670, 120, "mps");
label("ALT1", 600, 270, "alt1");
label(   "m", 670, 380, "m");
label("km/h", 670, 570, "kmph");
label("ALT3", 600, 630, "alt3");
label(   "m", 670, 750, "m");
label("TIME", 425, 790, "time");

var needle_t = needle();
var vario_t = variable( "0.0", 130, "vario");
var alt1_t = variable(    "0", 310, "alt");
var speed_t = variable( "0.0", 490, "speed");
var alt3_ = variable(     "0", 670, "alt");

var time_t = variable("00:00:00", 840, "time");

image("Models/display.png", 0, 0, 795, 1024);

var rtimer = maketimer(0.2, func {
  var vario_ms = (getprop("velocities/vertical-speed-fps") or 0)* 0.3048;
  vario_t.setText(sprintf("%.1f", vario_ms));
  draw_vario(needle_t, vario_ms);

  var glvl = 0;
  var PA_m = (getprop("/instrumentation/altimeter/pressure-alt-ft") or 0) * 0.3048;
  alt1_t.setText(sprintf("%i", PA_m));

  var speed_kmph = (getprop("velocities/airspeed-kt") or 0)* 1.852;
  speed_t.setText(sprintf("%.1f", speed_kmph));

  var hour = getprop("/sim/time/real/hour");
  var minute = getprop("/sim/time/real/minute");
  var second = getprop("/sim/time/real/second");
  time_t.setText(sprintf("%02i:%02i:%02i", hour, minute, second));
});
rtimer.start();
