// original code from
// http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
// robot geometry
var e = 115.0;            // end effector
var f = 457.3;            // base
var re = 232.0;
var rf = 112.0;
var btf = 400.0;
var s = 3200.0;           // microsteps

// trigonometric constants
var sqrt3 = Math.sqrt(3.0);
var pi = Math.PI;//3.141592653;     // PI
var sin120 = sqrt3 / 2.0;
var cos120 = -0.5;
var tan60 = sqrt3;
var sin30 = 0.5;
var tan30 = 1.0 / sqrt3;

var no_error = 0;
var non_existing_povar_error = 1;

// forward kinematics: (thetaA, thetaB, thetaC) -> (x0, y0, z0)
// returned {error code, thetaA, thetaB, thetaC}
function forward(thetaA, thetaB, thetaC) {
    var x0 = 0.0;
    var y0 = 0.0;
    var z0 = 0.0;

    var t = (f - e) * tan30 / 2.0;
    var dtr = pi / 180.0;

    thetaA *= dtr;
    thetaB *= dtr;
    thetaC *= dtr;

    var y1 = -(t + rf * Math.cos(thetaA));
    var z1 = -rf * Math.sin(thetaA);

    var y2 = (t + rf * Math.cos(thetaB)) * sin30;
    var x2 = y2 * tan60;
    var z2 = -rf * Math.sin(thetaB);

    var y3 = (t + rf * Math.cos(thetaC)) * sin30;
    var x3 = -y3 * tan60;
    var z3 = -rf * Math.sin(thetaC);

    var dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    var w1 = y1 * y1 + z1 * z1;
    var w2 = x2 * x2 + y2 * y2 + z2 * z2;
    var w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    var a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    var b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    // y = (a2*z + b2)/dnm;
    var a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    var b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    // a*z^2 + b*z + c = 0
    var a = a1 * a1 + a2 * a2 + dnm * dnm;
    var b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    var c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

    // discriminant
    var d = b * b - 4.0 * a * c;
    if (d < 0.0) {
        // console.log("forward", 1, 0, 0, 0)
        return Array(non_existing_povar_error, 0, 0, 0); // non-existing povar. return error,x,y,z
    }

    z0 = -0.5 * (b + Math.sqrt(d)) / a;
    x0 = (a1 * z0 + b1) / dnm;
    y0 = (a2 * z0 + b2) / dnm;

    // console.log("forward", 0, x0, y0, z0)

    return Array(no_error, x0, y0, z0);// return error,x,y,z
}


// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
function delta_calcAngleYZ(x0, y0, z0) {
    var y1 = -0.5 * 0.57735 * f;    // f/2 * tan(30 deg)
    y0 -= 0.5 * 0.57735 * e;        // shift center to edge

    // z = a + b*y
    var aV = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0);
    var bV = (y1 - y0) / z0;

    // discriminant
    var dV = -(aV + bV * y1) * (aV + bV * y1) + rf * (bV * bV * rf + rf);
    if (dV < 0) return Array(non_existing_povar_error, 0); // non-existing povar.  return {error, theta}

    var yj = (y1 - aV * bV - Math.sqrt(dV)) / (bV * bV + 1); // choosing outer povar
    var zj = aV + bV * yj;
    theta = Math.atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) ? 180.0 : 0.0);

    return Array(no_error, theta);  // return error, theta
}


// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)
// returned {error code, thetaA, thetaB, thetaC}
function inverse(x0, y0, z0) {
    var thetaA = 0;
    var thetaB = 0;
    var thetaC = 0;
    var status = delta_calcAngleYZ(x0, y0, z0);

    if (status[0] == no_error) {
        thetaA = status[1];
        status = delta_calcAngleYZ(
            x0 * cos120 + y0 * sin120,
            y0 * cos120 - x0 * sin120,
            z0,
            thetaB);  // rotate coords to +120 deg
    }
    if (status[0] == no_error) {
        thetaB = status[1];
        status = delta_calcAngleYZ(
            x0 * cos120 - y0 * sin120,
            y0 * cos120 + x0 * sin120,
            z0,
            thetaC);  // rotate coords to -120 deg
    }

    thetaC = status[1];

    return Array(status[0], thetaA, thetaB, thetaC);
}


function roundoff(x, y) {
    z = Math.pow(10, y);
    if (y == undefined) y = 3;
    return Math.round(x * z) / z;
}


function read_inputs() {
    e = parseFloat($('#e').val());      //End Effector radius (e)
    f = parseFloat($('#f').val());      //Base radius (f)
    re = parseFloat($('#re').val());    //Forearm length (re)
    rf = parseFloat($('#rf').val());    //Bicep length (rf)
    s = parseInt($('#s').val());        //Steps per turn
    btf = parseFloat($('#b').val());    //Base to floor distance (b)
}


function test_bounds() {
    read_inputs();

    var x_max = -e - f - re - rf;
    var y_max = x_max;
    var z_max = x_max;
    var x_min = -x_max;
    var y_min = -x_max;
    var z_min = -x_max;
    var sd = 360.0 / s;
    var _x, _y, _z;

    // find extents
    for (_z = 0; _z < s; ++_z) {
        r = forward(_z * sd, _z * sd, _z * sd);

        if (r[0] == 0) {
            if (z_min > r[3]) z_min = r[3];
            if (z_max < r[3]) z_max = r[3];
        }
    }
    if (z_min < -btf) z_min = -btf;
    if (z_max < -btf) z_max = -btf;

    var z_middle = (z_max + z_min) * 0.5;
    // $('#output').append("<p>(" + maxz + "," + minz + "," + middlez + ")</p>");
    var original_dist = (z_max - z_middle);
    var dist = original_dist * 0.5;
    var sum = 0;
    var r = Array(8);
    var mint1 = 360;
    var maxt1 = -360;
    var mint2 = 360;
    var maxt2 = -360;
    var mint3 = 360;
    var maxt3 = -360;

    var index = 0;

    do {
        sum += dist;
        r[0] = inverse(+sum, +sum, z_middle + sum);
        r[1] = inverse(+sum, -sum, z_middle + sum);
        r[2] = inverse(-sum, -sum, z_middle + sum);
        r[3] = inverse(-sum, +sum, z_middle + sum);
        r[4] = inverse(+sum, +sum, z_middle - sum);
        r[5] = inverse(+sum, -sum, z_middle - sum);
        r[6] = inverse(-sum, -sum, z_middle - sum);
        r[7] = inverse(-sum, +sum, z_middle - sum);

        if (r[0][0] != no_error || r[1][0] != no_error || r[2][0] != no_error || r[3][0] != no_error ||
            r[4][0] != no_error || r[5][0] != no_error || r[6][0] != no_error || r[7][0] != no_error) {
            sum -= dist;
            dist *= 0.5;
        } else {
            for (i = 0; i < 8; ++i) {
                if (mint1 > r[i][1]) mint1 = r[i][1];
                if (maxt1 < r[i][1]) maxt1 = r[i][1];
                if (mint2 > r[i][2]) mint2 = r[i][2];
                if (maxt2 < r[i][2]) maxt2 = r[i][2];
                if (mint3 > r[i][3]) mint3 = r[i][3];
                if (maxt3 < r[i][3]) maxt3 = r[i][3];
            }
        }

        // console.log("original_dist= ", original_dist, " , sum= ", sum, ", dist=", dist)

    } while (original_dist > sum && dist > 0.1);

    // console.log('index= ', index)

    var home = forward(0, 0, 0);
    // console.log('sum', -sum, sum)
    $('#center').html("(0,0," + roundoff(z_middle, 3) + ")");
    $('#home').html("(0,0," + roundoff(home[3], 3) + ")");

    // $('#bounds').html("X=[" + roundoff(-sum, 3) + ", " + roundoff(sum, 3) + "] mm"
    //     + "<br />Y=[" + roundoff(-sum, 3) + ", " + roundoff(sum, 3) + "] mm"
    //     + "<br />Z=[" + roundoff(middlez - sum, 3) + ", " + roundoff(middlez + sum, 3) + "] mm");
    $('#bounds').html(`X=[${roundoff(-sum, 3)}, ${roundoff(sum, 3)}] mm<br />\
                       Y=[${roundoff(-sum, 3)}, ${roundoff(sum, 3)}] mm<br />\
                       Z=[${roundoff(z_middle - sum, 3)}, ${roundoff(z_middle + sum, 3)}] mm`);

    // $('#limits').text("A=[" + roundoff(mint1, 2) + ", " + roundoff(maxt1, 2)
    //     + "]<br />B=[" + roundoff(mint2, 2) + ", " + roundoff(maxt2, 2)
    //     + "]<br />C=[" + roundoff(mint3, 2) + ", " + roundoff(maxt3, 2)) + "]";
    $('#limits').html(`A=[${roundoff(mint1, 2)}, ${roundoff(maxt1, 2)}]°<br />\
                       B=[${roundoff(mint2, 2)}, ${roundoff(maxt2, 2)}]°<br />\
                       C=[${roundoff(mint3, 2)}, ${roundoff(maxt3, 2)}]°`)

    // resolution?  
    r1 = forward(0, 0, 0);
    r2 = forward(sd, 0, 0);

    _x = (r1[1] - r2[1]);
    _y = (r1[2] - r2[2]);
    sum = Math.sqrt(_x * _x + _y * _y);

    $('#res').html("+/-" + roundoff(sum, 3) + "mm");
}


function test_fk_ik_match() {
    read_inputs();
    theta1 = parseFloat($('#t1').val());
    theta2 = parseFloat($('#t2').val());
    theta3 = parseFloat($('#t3').val());
    results1 = forward(theta1, theta2, theta3);
    results2 = inverse(results1[1], results1[2], results1[3]);

    $('#x').val(roundoff(results1[1], 3));
    $('#y').val(roundoff(results1[2], 3));
    $('#z').val(roundoff(results1[3], 3));
}


function test_fk() {
    read_inputs();
    theta1 = parseFloat($('#t1').val());
    theta2 = parseFloat($('#t2').val());
    theta3 = parseFloat($('#t3').val());
    results1 = forward(theta1, theta2, theta3);
    $('#x').val(roundoff(results1[1], 3));
    $('#y').val(roundoff(results1[2], 3));
    $('#z').val(roundoff(results1[3], 3));
}


function test_ik() {
    read_inputs();
    x = parseFloat($('#x').val());
    y = parseFloat($('#y').val());
    z = parseFloat($('#z').val());
    results2 = inverse(x, y, z);
    $('#t1').val(roundoff(results2[1], 3));
    $('#t2').val(roundoff(results2[2], 3));
    $('#t3').val(roundoff(results2[3], 3));
}


function test() {
    test_bounds();
    test_fk_ik_match();
}


$(document).ready(function () {
    test();
});