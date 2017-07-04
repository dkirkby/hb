coord = function(x, y) {
    return "" + x + "," + y;
}

clip = function(x, xlo, xhi) {
    return (x < xlo) ? xlo : ((x > xhi) ? xhi : x);
}

function Boundary(inner, vertices) {
    this.inner = inner;
    this.vertices = vertices;
    this.spring_constant = 100.0;
    this.d2max = 25.0 * 25.0;
}

Boundary.prototype.draw = function(axes, classname) {
    this.axes = axes;
    var points_str = "";
    for(var i = 0; i < this.vertices.length; i++) {
        points_str += coord(this.vertices[i][0], this.vertices[i][1])
            + " ";
    }
    points_str += coord(this.vertices[0][0], this.vertices[0][1]);
    axes.append("polyline")
        .attr("points", points_str)
        .attr("class", "boundary " + classname);
}

Boundary.prototype.restoring_force = function(point, where) {
    // Find which edge was breached.
    var x1, x2, y1, y2, s, dx, dy, d2, cross, scale;
    var x0 = point[0], y0 = point[1], d2min = Number.MAX_VALUE,
        fx=0.0, fy=0.0;
    for(var i1 = 0; i1 < this.vertices.length; i1++) {
        i2 = (i1 + 1) % this.vertices.length;
        x1 = this.vertices[i1][0];
        x2 = this.vertices[i2][0];
        y1 = this.vertices[i1][1];
        y2 = this.vertices[i2][1];
        s = ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) /
            ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        // Point of closest approach must lie on (x1,y1) -> (x2,y2).
        if((s < 0) || (s >= 1)) continue;
        // Calculate vector (xp-x0,yp-y0) pointing to the point
        // of closest approach.
        dx = (1.0 - s) * x1 + s * x2 - x0;
        dy = (1.0 - s) * y1 + s * y2 - y0;
        d2 = dx * dx + dy * dy;
        // Keep track of the force due to the edge i1->i2 with the
        // smallest distance of closest approach.
        if(d2 < d2min) {
            d2min = d2;
            where[0] = i1;
            where[1] = i2;
            fx = this.spring_constant * dx;
            fy = this.spring_constant * dy;
            // Limit the maximum restoring force.
            if(d2 > this.d2max) {
                var scale = Math.sqrt(this.d2max / d2);
                fx *= scale;
                fy *= scale;
            }
        }
    }
    return [fx, fy];
}

function HouseBoat() { }

HouseBoat.prototype.initialize = function(axes) {
    // Initialize boat geometry.
    this.deg2rad = Math.PI / 180;
    this.length = 100;
    this.width = 60;
    // Initialize boat state.
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 45.0;
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;
    this.throttle = 0.0;
    this.steering = 0.0;
    this.phi = 0.0;
    this.damage = [0.0, 0.0, 0.0, 0.0];
    // Initialize an array of corner locations.
    this.corners = [[0.,0.], [0.,0.], [0.,0.], [0.,0.]];
    this.update_corners();
    this.boundary = new Boundary(false, this.corners);
    // Create boat's visual representation.
    this.boat = axes.append("g");
    this.boat.append("rect")
        .attr("width", "" + this.length)
        .attr("height", "" + this.width)
        .attr("transform", "translate(" +
            coord(-0.5 * this.length, -0.5 * this.width) + ")")
        .attr("class", "boat");
    var wake_group = this.boat.append("g")
        .attr("transform", "translate(-50, 0)");
    this.wake = wake_group.append("polyline")
        .attr("points", "-50,-15 0,0 -50,15")
        .attr("transform", "rotate(0) scale(-0.5)")
        .attr("class", "wake");
    this.corner_circles = [];
    var L = 0.5 * this.length, W = 0.5 * this.width;
    var xy = [[-L, -W], [-L, +W], [+L, +W], [+L, -W]];
    for(var i = 0; i < 4; i++) {
        var circle = this.boat.append("circle")
            .attr("class", "damage")
            .attr("cx", xy[i][0])
            .attr("cy", xy[i][1])
            .attr("r", 8);
        this.corner_circles.push(circle);
    }
    this.draw();
    // Initialize model constants.
    this.max_thrust = 2.0;
    this.drag_p = 0.1;
    this.drag_t = 2.0;
    this.drag_r = 0.2;
    this.leverarm = 1.0;
}

HouseBoat.prototype.update = function(
    throttle, steering, external_force, external_torque) {
    dt = 0.02;
    this.throttle = throttle;
    this.steering = steering;
    this.phi = 90.0 * steering;
    // Initialize a unit vector aligned with the pontoons.
    var theta = this.theta * this.deg2rad;
    var nx = Math.cos(theta), ny = Math.sin(theta);
    // Split the velocity (vx,vy) into components parallel
    // and perpendicular to the pontoons.
    var dotprod = this.vx * nx + this.vy * ny;
    var vpx = dotprod * nx;
    var vpy = dotprod * ny;
    var vtx = this.vx - vpx;
    var vty = this.vy - vpy;
    // Calculate the combined drag force.
    var drag_x = -this.drag_p * vpx - this.drag_t * vtx;
    var drag_y = -this.drag_p * vpy - this.drag_t * vty;
    // Calculate the thrust force on the COM.
    var phi = this.phi * this.deg2rad;
    var angle = theta + phi;
    var thrust_mag = this.max_thrust * this.throttle;
    var thrust_x = thrust_mag * Math.cos(angle);
    var thrust_y = thrust_mag * Math.sin(angle);
    // Calculate the net force on the COM
    var Fx = thrust_x + drag_x + external_force[0];
    var Fy = thrust_y + drag_y + external_force[1];
    // Update the position and bearing using the present motion.
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    this.theta += this.omega * dt;
    //Update the COM velocity (assuming mass=1).
    this.vx += Fx * dt;
    this.vy += Fy * dt;
    // Calculate the torque about the COM due to the thrust.
    var tprop = -thrust_mag * this.leverarm * Math.sin(phi);
    // Calculate the torque about the COM due to rotational drag.
    var tdrag = -this.omega * this.drag_r;
    // Calculate the net torque about the COM.
    var tnet = tprop + tdrag + external_torque;
    // Update the angular velocity about the COM (assuming I=1).
    this.omega += tnet * dt;
    // Update corner coordinates.
    this.update_corners();
}

HouseBoat.prototype.update_corners = function() {
    var theta = this.theta * this.deg2rad;
    var nx = Math.cos(theta), ny = -Math.sin(theta);
    var L = 0.5 * this.length, W = 0.5 * this.width;
    this.corners[0][0] = this.x - nx * L - ny * W;
    this.corners[0][1] = this.y + ny * L - nx * W;
    this.corners[1][0] = this.x - nx * L + ny * W;
    this.corners[1][1] = this.y + ny * L + nx * W;
    this.corners[2][0] = this.x + nx * L + ny * W;
    this.corners[2][1] = this.y - ny * L + nx * W;
    this.corners[3][0] = this.x + nx * L - ny * W;
    this.corners[3][1] = this.y - ny * L - nx * W;
}

HouseBoat.prototype.draw = function() {
    this.boat
        .attr("transform",
            "translate(" + coord(this.x,this.y) + ") rotate(" +
            this.theta + ",0,0)");
    if(Math.abs(this.throttle) < 0.1) {
        wake_scale = 0.1;
    }
    else {
        wake_scale = this.throttle;
    }
    this.wake
        .attr("transform", "rotate(" + this.phi + ") scale(" +
            wake_scale + ")");
    for(var i = 0; i < 4; i++) {
        this.corner_circles[i].attr("fill-opacity", "" + this.damage[i]);
    }
}

function Simulator() {
}

Simulator.prototype.initialize = function(current_x, current_y) {
    this.current_x = current_x;
    this.current_y = current_y;
    this.svg = d3.select("svg");
    this.window_width = +this.svg.attr("width");
    this.window_height = +this.svg.attr("height");
    var wby2 = 0.5 * this.window_width, hby2 = 0.5 * this.window_height;
    // Set coordinate system with origin at center of the window,
    // x increasing to the right, and y increasing upwards.
    this.axes = this.svg.append("g")
        .attr("transform", "translate(" + coord(wby2, hby2) +
            ") scale(1, -1)");
    // Initialize throttle and steering controls.
    var radius = 16.0;
    this.control_radius = radius;
    // Scale control range [-1,+1] to display pixels.
    this.throttle_zero = hby2;
    this.throttle_max = -(hby2 - radius);
    this.steering_zero = wby2 - radius;
    this.steering_max = wby2 - 2 * radius;
    // Draw guide lines for each control.
    this.svg.append("line")
        .attr("x1", this.window_width - radius)
        .attr("x2", this.window_width - radius)
        .attr("y1", radius)
        .attr("y2", this.window_height - radius)
        .attr("class", "guide");
    this.svg.append("line")
        .attr("x1", radius)
        .attr("x2", this.window_width - 3 * radius)
        .attr("y1", this.window_height - radius)
        .attr("y2", this.window_height - radius)
        .attr("class", "guide");
    // Draw circles representing the current control setting.
    this.throttle_display = this.svg.append("circle")
        .attr("cx", this.window_width - radius)
        .attr("cy", this.throttle_zero)
        .attr("r", radius)
        .attr("class", "control");
    this.steering_display = this.svg.append("circle")
        .attr("cx", this.steering_zero)
        .attr("cy", this.window_height - radius)
        .attr("r", radius)
        .attr("class", "control");
    // Draw center marks for each control.
    var w = 0.2 * radius, h = 0.6 * radius;
    this.svg.append("polygon")
        .attr("points",
            coord(this.steering_zero - w, this.window_height) + " " +
            coord(this.steering_zero, this.window_height - h) + " " +
            coord(this.steering_zero + w, this.window_height))
        .attr("class", "marker");
    this.svg.append("polygon")
        .attr("points",
            coord(this.window_width, this.throttle_zero - w) + " " +
            coord(this.window_width - h, this.throttle_zero) + " " +
            coord(this.window_width, this.throttle_zero + w))
        .attr("class", "marker");
    // Draw invisible regions where control clicks are detected.
    this.throttle_input = this.svg.append("rect")
        .attr("x", this.window_width - 2 * radius)
        .attr("y", 0)
        .attr("width", 2 * radius)
        .attr("height", this.window_height)
        .attr("class", "control-input");
    this.steering_input = this.svg.append("rect")
        .attr("x", 0)
        .attr("y", this.window_height - 2 * radius)
        .attr("width", this.window_width - 2 * radius)
        .attr("height", 2 * radius)
        .attr("class", "control-input");
    // Draw timer in upper left.
    this.elapsed = 0.0;
    this.timer_text = this.svg.append("text")
        .attr("x", "100")
        .attr("y", "30")
        .text("0.0")
        .attr("class", "timer");
    // Capture arrow keypress events.
    this.right = false;
    this.left = false;
    this.up = false;
    this.down = false;
    var self = this;
    d3.select("body")
        .on("keydown", function() {
            var code = d3.event.keyCode;
            if(code == 39) {
                self.right = true;
            }
            else if(code == 37) {
                self.left = true;
            }
            else if(code == 38) {
                self.up = true;
            }
            else if(code == 40) {
                self.down = true;
            }
        })
        .on("keyup", function() {
            var code = d3.event.keyCode;
            if(code == 39) {
                self.right = false;
            }
            else if(code == 37) {
                self.left = false;
            }
            else if(code == 38) {
                self.up = false;
            }
            else if(code == 40) {
                self.down = false;
            }
        });
    // Capture mouse clicks in control inputs.
    this.steering_input
        .on("mousedown touchstart", function() {
            d3.event.preventDefault();
            d3.event.stopPropagation();
            var x;
            if(d3.event.type == "mousedown") {
                x = d3.mouse(this)[0];
            }
            else {
                x = d3.touches(this)[0][0];
            }
            var right = (x >= self.steering_display.attr("cx"));
            self.right = right;
            self.left = !right;
        })
        .on("mouseup touchend", function() {
            d3.event.preventDefault();
            d3.event.stopPropagation();
            self.left = false;
            self.right = false;
        });
    this.throttle_input
        .on("mousedown touchstart", function() {
            d3.event.preventDefault();
            d3.event.stopPropagation();
            var y;
            if(d3.event.type == "mousedown") {
                y = d3.mouse(this)[1];
            }
            else {
                y = d3.touches(this)[0][1];
            }
            var up = (y < self.throttle_display.attr("cy"));
            self.up = up;
            self.down = !up;
        })
        .on("mouseup touchend", function() {
            d3.event.preventDefault();
            d3.event.stopPropagation();
            self.up = false;
            self.down = false;
        });

    // Create a boundary limiting the boat to the visible area.
    limits = new Boundary(true,
        [[-wby2, -hby2], [-wby2, hby2], [wby2, hby2], [wby2, -hby2]]);
    limits.draw(this.axes, "water");

    this.houseboat = new HouseBoat();
    this.houseboat.initialize(this.axes);

    // Create a boundary for navigating around.
    var x1=-350, x2=-150, y1=-150, y2=100;
    var L=this.houseboat.length, W=this.houseboat.width;
    dock1 = new Boundary(false,
        [[x1, y1], [x1,y2-2.5*W], [x1+1.2*L,y2-2.5*W], [x1+1.2*L,y2-W],
         [x1,y2-W], [x1, y2], [x2, y2], [x2,y1]]);
    dock1.draw(this.axes, "dock");

    x1 = 100, x2= 350, x3=310, y1=120, y2=160, y3=-150;
    dock2 = new Boundary(false,
        [[x1, y1], [x1, y2], [x2, y2], [x2, y3], [x3, y3], [x3, y1]]);
    dock2.draw(this.axes, "dock");

    x1=50, x2=80, y1=-200, y2=-170;
    buoy = new Boundary(false,
        [[x1,y1], [x1,y2], [x2,y2], [x2,y1]]);
    buoy.draw(this.axes, "dock");

    this.boundaries = [ limits, dock1, dock2, buoy ];
}

Simulator.prototype.run = function() {
    var ival = 10; // milliseconds
    // Start the interval timer.
    var self = this;
    d3.interval(function() {
        // Adjust the throttle.
        var throttle = self.houseboat.throttle;
        var throttle_adjust = +1. * self.up - 1. * self.down;
        throttle = clip(throttle + 0.01 * throttle_adjust, -1, +1);
        // Adjust the steering.
        var steering = self.houseboat.steering;
        var steering_adjust = +1. * self.right - 1. * self.left;
        steering = clip(steering + 0.005 * steering_adjust, -1, +1);
        // Update controls display.
        self.throttle_display.classed("active", throttle_adjust != 0);
        self.steering_display.classed("active", steering_adjust != 0);
        self.throttle_display
            .attr("cy", self.throttle_zero + throttle * self.throttle_max);
        self.steering_display
            .attr("cx", self.steering_zero + steering * self.steering_max);
        // Test for any boat-boundary collisions.
        var corners = self.houseboat.corners, where = [0,0];
        var external_torque = 0.0,
            external_force = [self.current_x, self.current_y];
        for(var i = 0; i < self.boundaries.length; i++) {
            var boundary = self.boundaries[i];
            var inner = boundary.inner;
            // Loop over corners of the boat.
            for(var j = 0; j < corners.length; j++) {
                var inside = d3.polygonContains(
                    boundary.vertices, corners[j]);
                if((inner && !inside) || (!inner && inside)) {
                    // A corner of the boundary is inside the boat.
                    var force = boundary.restoring_force(
                        corners[j], where);
                    external_force[0] += force[0];
                    external_force[1] += force[1];
                    // Calculate torque about COM.
                    var rx = corners[j][0] - self.houseboat.x;
                    var ry = corners[j][1] - self.houseboat.y;
                    external_torque +=
                        0.01 * (rx * force[1] - ry * force[0]);
                    // Calculate damage in proportion to |force|.
                    damage = 0.0002 * Math.sqrt(
                        force[0] * force[0] + force[1] * force[1]);
                    self.houseboat.damage[j] =
                        clip(self.houseboat.damage[j] + damage, 0, 1);
                }
            }
            if(inner) continue;
            // Loop over vertices of this boundary.
            for(var k = 0; k < boundary.vertices.length; k++) {
                var inside = d3.polygonContains(
                    self.houseboat.corners, boundary.vertices[k]);
                if(!inside) continue;
                var force = self.houseboat.boundary.restoring_force(
                    boundary.vertices[k], where);
                external_force[0] -= force[0];
                external_force[1] -= force[1];
                // Calculate damage in proportion to |force|.
                damage = 0.0002 * Math.sqrt(
                    force[0] * force[0] + force[1] * force[1]);
                // Calculate torque about COM.
                var rx = boundary.vertices[k][0] - self.houseboat.x;
                var ry = boundary.vertices[k][1] - self.houseboat.y;
                external_torque -=
                    0.01 * (rx * force[1] - ry * force[0]);
                // Add damage to both endpoints of the hit edge.
                self.houseboat.damage[where[0]] =
                    clip(self.houseboat.damage[where[0]] + damage, 0, 1);
                self.houseboat.damage[where[1]] =
                    clip(self.houseboat.damage[where[1]] + damage, 0, 1);
            }
        }
        self.houseboat.update(
            throttle, steering, external_force, external_torque);
        self.houseboat.draw();
        // Update the timer.
        self.elapsed += 0.001 * ival;
        self.timer_text.text(self.elapsed.toFixed(1));
    }, ival);
}

var sim = new Simulator();

window.addEventListener('load', function() {
    // Simulator parameters are (x,y) components of "current", which is
    // implemented as a constant external force, so does not really model
    // current (or wind) correctly.
    sim.initialize(0, 0);
    sim.run();
});
