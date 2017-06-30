function HouseBoat() {
    console.log("Created new HouseBoat");
}

HouseBoat.prototype.initialize = function() {
    console.log("HouseBoat.initialize()");
    var svg = d3.select("svg"),
        width = +svg.attr("width"),
        height = +svg.attr("height");
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
}

HouseBoat.prototype.run = function() {
    // Start the interval timer.
    var self = this;
    this.interval_id = setInterval(function() {
        var turn_adjust = +1. * self.right - 1 * self.left;
        var throttle_adjust = +1. * self.up - 1. * self.down;
        self.message.text("turn " + turn_adjust + " throttle " + throttle_adjust);
    }, 200 /* ms */);
}

$(function() {
    // DOM is ready.
    $("#message").text("Initializing");
    var hb = new HouseBoat();
    hb.initialize();
    $("#message").text("Running");
    hb.run();
});
