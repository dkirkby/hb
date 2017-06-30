function HouseBoat() {
    console.log("Created new HouseBoat");
}

HouseBoat.prototype.initialize = function() {
    console.log("HouseBoat.initialize()");
    var svg = d3.select("svg"),
        width = +svg.attr("width"),
        height = +svg.attr("height");

    /**
    this.width = 30;
    this.length = 50;
    this.x = 0.5 * width;
    this.y = 0.5 * height;
    this.theta = 0.0;

    svg.selectAll("rect")
            .data([this])
        .enter().append("rect")
            .attr("x", 0.5 * (width - this.width))
            .attr("y", 0.5 * (height - this.length))
            .attr("width", this.width)
            .attr("height", this.length)
            .style("fill", "black")
            .style("opacity", 0.5);
    **/
}

$(function() {
    // DOM is ready.
    $( "p" ).text( "Initializing" );
/*
  });

// Need scripts to load before going further.
$(window).on("load", function(){
*/
    var hb = new HouseBoat();
    hb.initialize();
    $( "p" ).text( "Running" );
});
