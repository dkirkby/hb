var svg = d3.select("svg"),
    width = +svg.attr("width"),
    height = +svg.attr("height");

var hblength = 50;
var hbwidth = 30;

svg.selectAll("rect")
        .data([0])
    .enter().append("rect")
        .attr("x", 0.5 * (width - hbwidth))
        .attr("y", 0.5 * (height - hblength))
        .attr("width", hbwidth)
        .attr("height", hblength)
        .style("fill", "black")
        .style("opacity", 0.5);
