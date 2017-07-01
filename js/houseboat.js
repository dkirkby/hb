function HouseBoat() {
    console.log("Created new HouseBoat");
}

HouseBoat.prototype.initialize = function() {
    console.log("HouseBoat.initialize()");
    this.svg = d3.select("svg");
    this.window_width = +this.svg.attr("width");
    this.window_height = +this.svg.attr("height");
    this.message = $("#message");
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
        //console.log('tick', elapsed, self.throttle, self.steering);
        self.throttle_display //.transition(self.trans)
            .attr("cy", 0.5 * (self.window_height - self.throttle_max * self.throttle));
        self.steering_display //.transition(self.trans)
            .attr("cx", 0.5 * (self.window_width + self.steering_max * self.steering));
    }, ival);
}

$(function() {
    // DOM is ready.
    $("#message").text("Initializing");
    var hb = new HouseBoat();
    hb.initialize();
    $("#message").text("Running");
    hb.run();
});
