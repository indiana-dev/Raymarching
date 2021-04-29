#version 300 es
precision highp float;

uniform float iTime;
uniform float MAX_DIST;

uniform mat4 viewMatrix;
uniform vec3 camPos;

uniform vec3 lightPos;
uniform float penumbra;

uniform float debugA;
uniform float debugB;

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

// Maximum/minumum elements of a vector
float vmax(vec2 v) {
	return max(v.x, v.y);
}

float vmax(vec3 v) {
	return max(max(v.x, v.y), v.z);
}

float vmax(vec4 v) {
	return max(max(v.x, v.y), max(v.z, v.w));
}

float vmin(vec2 v) {
	return min(v.x, v.y);
}

float vmin(vec3 v) {
	return min(min(v.x, v.y), v.z);
}

float vmin(vec4 v) {
	return min(min(v.x, v.y), min(v.z, v.w));
}

// Shortcut for 45-degrees rotation
void pR45(inout vec2 p) {
	p = (p + vec2(p.y, -p.x))*sqrt(0.5);
}

// Space manipulation

float pMod1(inout float p, float size) {
	float halfsize = size*0.5;
	float c = floor((p + halfsize)/size);
	p = mod(p + halfsize, size) - halfsize;
	return c;
}

float fOpDifferenceColumns(float a, float b, float r, float n) {
	a = -a;
	float m = min(a, b);
	//avoid the expensive computation where not needed (produces discontinuity though)
	if ((a < r) && (b < r)) {
		vec2 p = vec2(a, b);
		float columnradius = r*sqrt(2.)/n/2.0;
		columnradius = r*sqrt(2.)/((n-1.)*2.+sqrt(2.));

		pR45(p);
		p.y += columnradius;
		p.x -= sqrt(2.)/2.*r;
		p.x += -columnradius*sqrt(2.)/2.;

		if (mod(n,2.) == 1.) {
			p.y += columnradius;
		}
		pMod1(p.y,columnradius*2.);

		float result = -length(p) + columnradius;
		result = max(result, p.x);
		result = min(result, a);
		return -min(result, b);
	} else {
		return -m;
	}
}

// Distance functions

float fSphere(vec3 p, float r) {
    vec3 c = vec3(5.+iTime*5.);
    vec3 q = mod(p+0.5*c,c)-0.5*c;
	return length(q) - r;
}

float fBox(vec3 p, vec3 b) {
	vec3 d = abs(p) - b;
	return length(max(d, vec3(0))) + vmax(min(d, vec3(0)));
}

float fBox2(vec2 p, vec2 b) {
	vec2 d = abs(p) - b;
	return length(max(d, vec2(0))) + vmax(min(d, vec2(0)));
}

float fCylinder(vec3 p, float r, float height) {
	float d = length(p.xz) - r;
	d = max(d, abs(p.y) - height);
	return d;
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
    // float i = pMod1(p.z, 20.);

    // float wall = fBox2(p.xy, vec2(1., 15.));

    // p.z = abs(p.z)-3.;
    // p.z = abs(p.z)+2.;

    // float box = fBox(p, vec3(3., 9., 4.));
    // p.y -= 9.;
    // float cylinder = fCylinder(p.yxz, 4., 3.);

    // float window = min(box, cylinder);

    // return fOpDifferenceColumns(wall, window, 0.6, 3.);
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
    
    vec3 light1Pos = lightPos;
    vec3 light1Intensity = vec3(debugA);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);
    
    // vec3 light2Pos = vec3(2.0 * sin(0.37 * iTime),
    //                       -200.0,
    //                       2.0);
    // vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
    
    // color += phongContribForLight(k_d, k_s, alpha, p, eye,
    //                               light2Pos,
    //                               light2Intensity);    
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

/**
 * distToScene: the distance from the hit point to the scene excluding the plane
 * distToPoint: the distance from the camera to the hit point
 * frequency: the frequency of the isolines
 * thickness: the thickness of the isolines
 * smoothness: the range that will be used in smoothstep to smooth the border of the isolines
 * minDistance: the minimum distance at which the lines should appear
 * maxDistance: the maximum distance at which the lines should appear 
 */
vec3 createIsolines(float distToScene, float distToPoint, float frequency, float thickness, float smoothness, float minDistance, float maxDistance) {
    float fade = distToPoint < minDistance ? smoothstep(minDistance, 0., distToPoint) : smoothstep(minDistance, maxDistance, distToPoint);
    float c = 0.5+0.5*sin(distToScene*frequency);

    vec3 colStart = vec3(0.57, 0., 0.75);
    vec3 colMid = vec3(255./255., 216./255., 101./255.);
    vec3 colEnd = vec3(0.97);
    float end1 = 10.;
    float end2 = 20.;

    vec3 base = distToScene < end1 ? mix(colStart, colMid, smoothstep(0., end1, distToScene)) : mix(colMid, colEnd, clamp(0., end2, smoothstep(0., end2, distToScene - end1)));
            
    vec3 col;
    thickness *= max(1.0, 0.25*distToPoint);
    col = mix(col, base, smoothstep(thickness, thickness + smoothness, c));
    col = mix(col, base, fade);

    return col;
}

float calculateShadow(vec3 ro, vec3 rd, float mint, float maxt)
{
    float res = 1.0;
    float ph = 1e20;
    for( float t=mint; t<maxt; )
    {
        float h = sceneSDF(ro + rd*t);
        if( h<EPSILON )
            return 0.0;
        float y = h*h/(2.0*ph);
        float d = sqrt(h*h-y*y);
        res = min( res, penumbra*d/max(0.0,t-y) );
        ph = h;
        t += h;
    }
    return res;
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
//float distToScene, float distToPoint, float frequency, float thickness, float smoothness, float minDistance, float maxDistance) {
        vec3 col = createIsolines(dstToScene, dist, 1., 0.001, 0.01, 80., 2000.);
        col *= createIsolines(dstToScene, dist, 10., 0.001, 0.01, 20., 80.);
        col *= createIsolines(dstToScene, dist, 100., 0.001, 0.01, 0., 20.);
        // vec3 col = createIsolines(dstToScene, dist, 10., 0.001, 0.01, 1., 200.);

        // col *= calculateShadow(p, normalize(lightPos - p), EPSILON*2., length(lightPos - p));

        // Gamma
        //col = pow(col, vec3(0.4545));
        
        pixel = vec4(col,1.);
        return;
    }

    // Didn't hit anything
    if (type < 0.) {
        pixel = vec4(.2, .2, .2, 1.);
		return;
    }
     
    vec3 K_a = vec3(0.2);
    vec3 K_d = vec3(p/100.);//mod(p.y + .5,10.) < 5. ? 1. : .2);
    vec3 K_s = vec3(1.0, 1.0, 1.0);
    float shininess = 10.;
    
    vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, camPos);
    // color *= calculateShadow(p, normalize(lightPos - p), EPSILON*2., length(lightPos - p));
    //color += vec3(0.05);
    pixel = vec4(color, 1.0);
}