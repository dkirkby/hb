function HouseBoat() {
    console.log("Created new HouseBoat");
}

HouseBoat.prototype.initialize = function() {
    this.svg = d3.select("svg");
    this.window_width = +this.svg.attr("width");
    this.window_height = +this.svg.attr("height");
    // Initialize boat state.
    this.x = 0.5 * this.window_width;
    this.y = 0.5 * this.window_height;
    this.theta = 0.0;
    this.phi = 0.0;
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;
    // Create boat's visual representation.
    this.boat = this.svg.append("g");
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
    this.draw_boat();
    // Capture arrow keypress events.
    this.right = false;
    this.left = false;
    this.up = false;
    this.down = false;
    var self = this;
    d3.select("body")
        // up=38, dn=40
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
                self.steering_display.attr("fill", "red");
            }
            if(self.up || self.down) {
                self.throttle_display.attr("fill", "red");
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
                self.steering_display.attr("fill", "green");
            }
            if(!self.up && !self.down) {
                self.throttle_display.attr("fill", "green");
            }
        });
        // Initialize throttle and steering controls.
        this.throttle = 0.0;
        this.steering = 0.0;
        var radius = 16;
        this.throttle_display = this.svg.append("circle")
            .attr("cx", self.window_width - radius)
            .attr("cy", 0.5 * this.window_height - radius)
            .attr("r", radius)
            .attr("fill", "green");
        this.steering_display = this.svg.append("circle")
            .attr("cx", 0.5 * this.window_width - radius)
            .attr("cy", self.window_height - radius)
            .attr("r", radius)
            .attr("fill", "green");
        this.throttle_max = self.window_height - 2 * radius;
        this.steering_max = self.window_width - 2 * radius;
        // Initialize constants.
        this.deg2rad = Math.PI / 180;
        this.max_thrust = 10.0;
        this.drag_p = 0.1;
        this.drag_t = 2.0;
        this.drag_r = 0.2;
        this.current_x = 0.0;
        this.current_y = 0.0;
        this.leverarm = 1.0;
}

HouseBoat.prototype.update_state = function() {
    dt = 0.1;
    // Initialize a unit vector aligned with the pontoons.
    var theta = this.theta * this.deg2rad;
    var nx = Math.cos(theta), ny = Math.sin(theta);
    // Split the velocity (vx,vy) into components parallel
    // and perpendicular to the pontoons.
    var vpx = this.vx * nx + this.vy * ny;
    var vpy = this.vy * nx - this.vx * ny;
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

HouseBoat.prototype.draw_boat = function() {
    this.boat
        .attr("transform",
            "translate(" + this.x + "," + this.y + ") rotate(" + (-this.theta) + ",0,0)");
    if(Math.abs(this.throttle) < 0.05) {
        wake_scale = 0.05;
    }
    else {
        wake_scale = this.throttle;
    }
    this.wake
        .attr("transform", "rotate(" + (-this.phi) + ") scale(" +
            wake_scale + ")");
}

HouseBoat.prototype.run = function() {
    var ival = 50; // milliseconds
    this.trans = d3.transition().duration(ival).ease(d3.easeLinear);
    // Start the interval timer.
    var self = this;
    //this.interval_id = setInterval(function() {
    d3.interval(function(elapsed) {
        var throttle_adjust = +1. * self.up - 1. * self.down;
        self.throttle += 0.01 * throttle_adjust;
        if(self.throttle > 1.0) {
            self.throttle = 1.0;
        }
        else if(self.throttle < -1.0) {
            self.throttle = -1.0;
        }
        var steering_adjust = +1. * self.right - 1. * self.left;
        self.steering += 0.01 * steering_adjust;
        if(self.steering > 1.0) {
            self.steering = 1.0;
        }
        else if(self.steering < -1.0) {
            self.steering = -1.0;
        }
        self.throttle_display //.transition(self.trans)
            .attr("cy", 0.5 * (self.window_height - self.throttle_max * self.throttle));
        self.steering_display //.transition(self.trans)
            .attr("cx", 0.5 * (self.window_width + self.steering_max * self.steering));
        self.phi = 90.0 * self.steering;
        self.update_state();
        self.draw_boat();
    }, ival);
}

var hb = new HouseBoat();

$(function() {
    // DOM is ready.
    hb.initialize();
    hb.run();
});
