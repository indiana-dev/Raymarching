#version 300 es
precision highp float;

uniform float iTime;
uniform float MAX_DIST;

uniform mat4 viewMatrix;
uniform vec3 camPos;

in vec2 uv;
out vec4 pixel;

/**
 * Part 2 Challenges
 * - Change the diffuse color of the sphere to be blue
 * - Change the specual color of the sphere to be green
 * - Make one of the lights pulse by having its intensity vary over time
 * - Add a third light to the scene
 */

const int MAX_MARCHING_STEPS = 1000;
const float MIN_DIST = 0.0;
// const float MAX_DIST = 100.0;
const float EPSILON = 0.001;

float fSphere(vec3 p, float r) {
    vec3 c = vec3(50);
    vec3 q = mod(p+0.5*c,c)-0.5*c;
	return length(p) - r;
}

/**
 * Signed distance function describing the scene.
 * 
 * Absolute value of the return value indicates the distance to the surface.
 * Sign indicates whether the point is inside or outside the surface,
 * negative indicating inside.
 */

float sceneSDF(vec3 p) {
    return fSphere(p, 1.);
}

// Plane with normal n (n is normalized) at some distance from the origin
float fPlane(vec3 p, vec3 n, float distanceFromOrigin) {
	return dot(p, n) + distanceFromOrigin;
}

/**
 * Return the shortest distance from the eyepoint to the scene surface along
 * the marching direction. If no part of the surface is found between start and end,
 * return end.
 * 
 * eye: the eye point, acting as the origin of the ray
 * marchingDirection: the normalized direction to march in
 * start: the starting distance away from the eye
 * end: the max distance away from the ey to march before giving up
 *
 * Returns: 
 * x: depth
 * y: info (-2: max distance, -1: max iterations, 0: hit, 1: plane)
 */
vec2 shortestDistanceToSurface(vec3 eye, vec3 marchingDirection, float start, float end) {
    float depth = start;
    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        vec3 point = eye + depth * marchingDirection;
        float dist = min(sceneSDF(point), point.y);

        if (point.y < EPSILON) {
            return vec2(depth, 1.);
        }


        if (dist < EPSILON) {
			return vec2(depth, 0.);
        }
        depth += dist;
        if (depth >= end) {
            return vec2(end, -2.);
        }
    }
    return vec2(end, -1.);
}
            

/**
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 */
// vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
//     vec2 xy = fragCoord - size / 2.0;
//     float z = size.y / tan(radians(fieldOfView) / 2.0);
//     return normalize(vec3(xy, -z));
// }

/**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

/**
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The vec3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,
                          vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));
    
    float dotLN = clamp(dot(L, N), 0., 1.);
    float dotRV = dot(R, V);
    
    if (dotLN < 0.0) {
        // Light not visible from this point on the surface
        return vec3(0.0, 0.0, 0.0);
    } 
    
    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Lighting via Phong illumination.
 * 
 * The vec3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    
    vec3 light1Pos = vec3(4.0 ,
                          200.0,
                          4.0 );
    vec3 light1Intensity = vec3(.6);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);
    
    vec3 light2Pos = vec3(2.0 * sin(0.37 * iTime),
                          -200.0,
                          2.0);
    vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light2Pos,
                                  light2Intensity);    
    return color;
}

/**
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 */
vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
    vec2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

void main()
{
	vec3 viewDir = rayDirection(45.0, vec2(1.), uv);    
    vec3 worldDir = (viewMatrix * vec4(viewDir, 0.0)).xyz;
    
    vec2 result = shortestDistanceToSurface(camPos, worldDir, MIN_DIST, MAX_DIST);
    float dist = result.x;
    float type = result.y;

    // The closest point on the surface to the eyepoint along the view ray
    vec3 p = camPos + dist * worldDir;

    // Hit floor plane
    if (type == 1.) {
        float dstToScene = sceneSDF(p);
        // vec3 colStart = vec3(0.57, 0., 0.75);
        // vec3 colMid = vec3(1., 0.75, 0.);
        // vec3 colEnd = vec3(0.8);
        // float dst = clamp(dstToScene / 10., 0., 2.);

        // vec3 planeColor = dst < 1. ? mix(colStart, colMid, dst) : mix(colMid, colEnd, dst - 1.);
        // pixel = vec4(planeColor, 1.);
        
        float col = 1.;
        col *= (mod(dstToScene, 1.) > 0.9 ? 0. : 1.) + dist/300.;
        col *= (mod(dstToScene, 10.) > 9. ? 0. : 1.) + dist/600.;
        col *= (mod(dstToScene, 100.) > 90. ? 0. : 1.) + dist/5000.;
        pixel = vec4(vec3(col),1.);
        return;
    }

    // Didn't hit anything
    if (type < 0.) {
        pixel = vec4(.2, .2, .2, 1.);
		return;
    }
     
    vec3 K_a = vec3(0.2);
    vec3 K_d = vec3(mod(p.y + .5,10.) < 5. ? 1. : .2);
    vec3 K_s = vec3(1.0, 1.0, 1.0);
    float shininess = 10.;
    
    vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, camPos);
    
    pixel = vec4(color, 1.0);
}