coord = function(x, y) {
    return "" + x + "," + y;
}

function Boundary(axes, fill, vertices) {
    this.vertices = vertices;
    var points_str = "";
    for(var i = 0; i < this.vertices.length; i++) {
        points_str += coord(this.vertices[i][0], this.vertices[i][1])
            + " ";
    }
    points_str += coord(this.vertices[0][0], this.vertices[0][1]);
    axes.append("polyline")
        .attr("points", points_str)
        .attr("stroke", "brown")
        .attr("stroke-width", "3.0")
        .attr("fill", fill);
}

function HouseBoat() { }

HouseBoat.prototype.initialize = function(axes) {
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
    // Create boat's visual representation.
    this.boat = axes.append("g");
    this.boat.append("rect")
        .attr("width", "100")
        .attr("height", "60")
        .attr("transform", "translate(-50, -30)")
        .attr("fill", "gray")
        .attr("stroke", "black");
    var wake_group = this.boat.append("g")
        .attr("transform", "translate(-50, 0)");
    this.wake = wake_group.append("polyline")
        .attr("points", "-50,-15 0,0 -50,15")
        .attr("transform", "rotate(0) scale(-0.5)")
        .attr("stroke", "red")
        .attr("fill", "none");
    this.draw();
    // Initialize model constants.
    this.deg2rad = Math.PI / 180;
    this.max_thrust = 10.0;
    this.drag_p = 0.1;
    this.drag_t = 2.0;
    this.drag_r = 0.2;
    this.current_x = 0.0;
    this.current_y = 0.0;
    this.leverarm = 1.0;
}

HouseBoat.prototype.update = function(throttle, steering) {
    dt = 0.1;
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
    var Fx = thrust_x + drag_x + this.current_x;
    var Fy = thrust_y + drag_y + this.current_y;
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
    var tnet = tprop + tdrag;
    // Update the angular velocity about the COM (assuming I=1).
    this.omega += tnet * dt;
}

HouseBoat.prototype.draw = function() {
    this.boat
        .attr("transform",
            "translate(" + coord(this.x,this.y) + ") rotate(" +
            this.theta + ",0,0)");
    if(Math.abs(this.throttle) < 0.05) {
        wake_scale = 0.05;
    }
    else {
        wake_scale = this.throttle;
    }
    this.wake
        .attr("transform", "rotate(" + this.phi + ") scale(" +
            wake_scale + ")");
}

function Simulator() {
}

Simulator.prototype.initialize = function() {
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
    this.throttle_display = this.svg.append("circle")
        .attr("cx", this.window_width - radius)
        .attr("cy", hby2)
        .attr("r", radius)
        .attr("stroke", "black")
        .attr("fill-opacity", "0.5")
        .attr("fill", "green");
    this.steering_display = this.svg.append("circle")
        .attr("cx", wby2)
        .attr("cy", this.window_height - radius)
        .attr("r", radius)
        .attr("stroke", "black")
        .attr("fill-opacity", "0.5")
        .attr("fill", "green");
    // Draw center marks for each control.
    var w = 0.2 * radius, h = 0.6 * radius;
    this.svg.append("polygon")
        .attr("points",
            coord(wby2 - w, this.window_height) + " " +
            coord(wby2, this.window_height - h) + " " +
            coord(wby2 + w, this.window_height))
        .attr("stroke", "black")
        .attr("fill", "blue");
    this.svg.append("polygon")
        .attr("points",
            coord(this.window_width, hby2 - w) + " " +
            coord(this.window_width - h, hby2) + " " +
            coord(this.window_width, hby2 + w))
        .attr("stroke", "black")
        .attr("fill", "blue");
    this.throttle_max = this.window_height - 2 * radius;
    this.steering_max = this.window_width - 2 * radius;

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
            if(self.right || self.left) {
                self.steering_display.attr("fill-opacity", "1.0");
            }
            if(self.up || self.down) {
                self.throttle_display.attr("fill-opacity", "1.0");
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
            if(!self.right && !self.left) {
                self.steering_display.attr("fill-opacity", "0.5");
            }
            if(!self.up && !self.down) {
                self.throttle_display.attr("fill-opacity", "0.5");
            }
        });

    this.boundaries = new Array();
    limits = new Boundary(this.axes, "lightblue",
        [[-wby2, -hby2], [-wby2, hby2], [wby2, hby2], [wby2, -hby2]]);

    this.houseboat = new HouseBoat();
    this.houseboat.initialize(this.axes);
}

Simulator.prototype.run = function() {
    var ival = 50; // milliseconds
    // Start the interval timer.
    var self = this;
    d3.interval(function(elapsed) {
        // Adjust the throttle.
        var throttle = self.houseboat.throttle;
        var throttle_adjust = +1. * self.up - 1. * self.down;
        throttle += 0.01 * throttle_adjust;
        if(throttle > 1.0) { throttle = 1.0; }
        else if(throttle < -1.0) { throttle = -1.0; }
        // Adjust the steering.
        var steering = self.houseboat.steering;
        var steering_adjust = +1. * self.right - 1. * self.left;
        steering += 0.01 * steering_adjust;
        if(steering > 1.0) { steering = 1.0; }
        else if(steering < -1.0) { steering = -1.0; }
        self.throttle_display //.transition(self.trans)
            .attr("cy",
            0.5 * (self.window_height - self.throttle_max * throttle));
        self.steering_display //.transition(self.trans)
            .attr("cx",
            0.5 * (self.window_width + self.steering_max * steering));
        self.houseboat.update(throttle, steering);
        self.houseboat.draw();
    }, ival);
}

var sim = new Simulator();

$(function() {
    // DOM is ready.
    sim.initialize();
    sim.run();
});
