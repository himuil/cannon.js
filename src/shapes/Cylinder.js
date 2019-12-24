module.exports = Cylinder;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');
var Quaternion = require('../math/Quaternion');
var ConvexPolyhedron = require('./ConvexPolyhedron');

/**
 * @class Cylinder
 * @constructor
 * @extends ConvexPolyhedron
 * @author schteppe / https://github.com/schteppe
 * @param {Number} radiusTop
 * @param {Number} radiusBottom
 * @param {Number} height
 * @param {Number} numSegments The number of segments to build the cylinder out of
 */
function Cylinder( radiusTop, radiusBottom, height , numSegments ) {
    var N = numSegments,
        cos = Math.cos,
        sin = Math.sin;

    var halfH = height/2;
    var vertices = [];
    var indices = [];
    var tf = [0];
    var bf = [1];
    var axes = [];
    var theta = Math.PI * 2 / N;
    for(var i = 0; i < N; i++){
        vertices.push(new Vec3(radiusTop * Math.cos(theta*i), halfH, radiusTop * Math.sin(theta*i)));
        vertices.push(new Vec3(radiusTop * Math.cos(theta*i), -halfH, radiusTop * Math.sin(theta*i)));

        if(i < N-1){
            indices.push([2*i+2, 2*i+3, 2*i+1,2*i]);
            tf.push(2*i+2);
            bf.push(2*i+3);
        }else{
            indices.push([0,1, 2*i+1, 2*i]);
        }

        if(N % 2 === 1 || i < N / 2){
            axes.push(new Vec3(cos(theta*(i+0.5)), 0, sin(theta*(i+0.5))));
        }
    }
    indices.push(bf);
    var temp = [];
    for(var i=0; i<tf.length; i++){
        temp.push(tf[tf.length - i - 1]);
    }
    indices.push(temp);
    axes.push(new Vec3(0,1,0));
    ConvexPolyhedron.call( this, vertices, indices, axes);
}

Cylinder.prototype = new ConvexPolyhedron();