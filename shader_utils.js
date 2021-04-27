function initWebGLContext() {
    const shader_program = initShaderProgram(calculation_vs, calculation_fs)
    gl.useProgram(shader_program)

    var position_buffer = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, position_buffer)
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1]), gl.STATIC_DRAW)

    var position_location = gl.getAttribLocation(shader_program, "a_position")
    gl.enableVertexAttribArray(position_location)
    gl.vertexAttribPointer(position_location, 2, gl.FLOAT, false, 0, 0);
    
    return shader_program
}

function initShaderProgram(vs_source, fs_source) {
    const vertex_shader = createShader(gl, gl.VERTEX_SHADER, vs_source)
    const fragment_shader = createShader(gl, gl.FRAGMENT_SHADER, fs_source)

    const shader_program = gl.createProgram()

    gl.attachShader(shader_program, vertex_shader)
    gl.attachShader(shader_program, fragment_shader)
    gl.linkProgram(shader_program)

    if (!gl.getProgramParameter(shader_program, gl.LINK_STATUS)) {
        console.log('Unable to initialize the shader program: ' + gl.getProgramInfoLog(shader_program))
        return null
    }
    
    return shader_program
}

function createShader(gl, type, source) {
    const shader = gl.createShader(type)

    gl.shaderSource(shader, source)
    gl.compileShader(shader)

    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        console.log('An error occurred compiling the shaders: ' + gl.getShaderInfoLog(shader))
        gl.deleteShader(shader)

        return null
    }
    
    return shader
}

function loadShaders() {
    $.ajax({url: 'calculation_frag.glsl', success: function(data) {
        calculation_fs = data
    }})

    $.ajax({url: 'calculation_vert.glsl', success: function(data) {
        calculation_vs = data
    }})
}