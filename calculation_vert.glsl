#version 300 es
in vec2 a_position;

out vec2 uv;

void main() {
    // convert from 0->1 to 0->2
    vec2 zeroToTwo = a_position * 2.0;

    // convert from 0->2 to -1->+1 (clipspace)
    vec2 clipSpace = zeroToTwo - 1.0;

    gl_Position = vec4(clipSpace, 0, 1);

    // pass the texCoord to the fragment shader
    // The GPU will interpolate this value between points.
    uv = a_position;
}