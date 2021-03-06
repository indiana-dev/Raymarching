window.onload = waitForShadersToInitialize

let shaders_loaded = false

let calculation_fs = null
let calculation_vs = null

loadShaders()

const gui = new dat.GUI()
let settings = {
    MAX_DIST: 1000,
    penumbra: 8,
    debugA: 0.2,
    debugB: 0.1
}

function initGUI() {
    gui.add(settings, 'MAX_DIST', 10, 50000)
    gui.add(settings, 'penumbra', 0.001, 100, 0.001)
    gui.add(settings, 'debugA', 0.001, 100., 0.001)
    gui.add(settings, 'debugB', 0.001, 1., 0.001)
}

class Light {
    constructor(x, y, z) {
        this.pos = vec3(x, y, z)
    } 
}

class Keyboard {
    constructor() {
        this.W = false
        this.A = false
        this.S = false
        this.D = false
        this.LEFT = false
        this.RIGHT = false
        this.UP = false
        this.DOWN = false
    }
}

class Camera {
    constructor(pos) {
        this.pos = pos
        this.yaw = -90
        this.pitch = 0
        this.speed = 1.
        this.rotationSpeed = 1

        this.updateViewMatrix()
    }

    updateViewMatrix() {
        this.front = normalize(vec3(
            Math.cos(radians(this.pitch)) * Math.cos(radians(this.yaw)),
            Math.sin(radians(this.pitch)),
            Math.cos(radians(this.pitch)) * Math.sin(radians(this.yaw))
        ))
        this.right = normalize(cross(this.front, vec3(0, 1, 0)))
        this.up = normalize(cross(this.right, this.front))

        this.viewMatrix = transpose(lookAt(this.pos, add(this.pos, this.front), this.up))
    }

    updatePosition() {
        const front = mult(vec3(...this.front), vec3(this.speed, this.speed, this.speed))
        if (keyboard.W) this.pos = add(this.pos, front)
        if (keyboard.S) this.pos = subtract(this.pos, front)
        if (keyboard.A) this.pos = subtract(this.pos, normalize(cross(front, this.up)))
        if (keyboard.D) this.pos = add(this.pos, normalize(cross(front, this.up)))
        
        if (keyboard.LEFT) this.yaw -= this.rotationSpeed
        if (keyboard.RIGHT) this.yaw += this.rotationSpeed
        if (keyboard.DOWN) this.pitch -= this.rotationSpeed
        if (keyboard.UP) this.pitch += this.rotationSpeed

        this.updateViewMatrix()
    }
}

let cam
let mouse = new Mouse()
let keyboard = new Keyboard()
let light = new Light(40, 30, 10)
let gl, shader_program, screen_size, uniforms

function waitForShadersToInitialize() {
    if(calculation_fs == null || calculation_vs == null) {
        requestAnimationFrame(waitForShadersToInitialize)
    } else {
        main()
    }
}

function main() {
    cam = new Camera(vec3(10.,2.,12))

    screen_size = 600

    const canvas = document.querySelector("#glCanvas")
    canvas.width = screen_size
    canvas.height = screen_size
    gl = canvas.getContext("webgl2")

    window.addEventListener('keydown', event => {
        if (event.code === 'KeyW')  keyboard.W = true
        else if (event.code === 'KeyA')  keyboard.A = true
        else if (event.code === 'KeyS')  keyboard.S = true
        else if (event.code === 'KeyD')  keyboard.D = true
        else if (event.code === 'ArrowLeft')  keyboard.LEFT = true
        else if (event.code === 'ArrowRight')  keyboard.RIGHT = true
        else if (event.code === 'ArrowUp')  keyboard.UP = true
        else if (event.code === 'ArrowDown')  keyboard.DOWN = true
    })

    window.addEventListener('keyup', event => {
        if (event.code === 'KeyW')  keyboard.W = false
        else if (event.code === 'KeyA')  keyboard.A = false
        else if (event.code === 'KeyS')  keyboard.S = false
        else if (event.code === 'KeyD')  keyboard.D = false
        else if (event.code === 'ArrowLeft')  keyboard.LEFT = false
        else if (event.code === 'ArrowRight')  keyboard.RIGHT = false
        else if (event.code === 'ArrowUp')  keyboard.UP = false
        else if (event.code === 'ArrowDown')  keyboard.DOWN = false
    })

    if (gl === null) {
        console.log("Unable to initialize WebGL. Your browser or machine may not support it.")
        return
    }

    shader_program = initWebGLContext()

    initUniforms()  
    initGUI()

    var then = 0;

    // Draw the scene repeatedly
    function render(now) {
        now *= 0.001;  // convert to seconds
        const deltaTime = now - then;
        then = now;

        draw();

        requestAnimationFrame(render);
    }
    requestAnimationFrame(render);
}

function initUniform(name) {
    uniforms[name] = gl.getUniformLocation(shader_program, name);
}

function initUniforms() {
    const initUniform = (name) => uniforms[name] = gl.getUniformLocation(shader_program, name)
    const uniform_names = [
        'iTime', 
        'MAX_DIST', 
        'viewMatrix', 
        'camPos', 
        'lightPos', 
        'penumbra',
        'debugA', 
        'debugB']

    uniforms = []

    for (let name of uniform_names) {
        initUniform(name)
    }
}

let iTime = 0.;
function setUniforms() {
    gl.uniform1f(uniforms.iTime, iTime);
    gl.uniform1f(uniforms.MAX_DIST, settings.MAX_DIST)
    gl.uniform1f(uniforms.penumbra, settings.penumbra)
    gl.uniform1f(uniforms.debugA, settings.debugA)
    gl.uniform1f(uniforms.debugB, settings.debugB)
    gl.uniformMatrix4fv(uniforms.viewMatrix, 0, flatten(cam.viewMatrix))
    gl.uniform3f(uniforms.camPos, ...cam.pos)
    gl.uniform3f(uniforms.lightPos, ...light.pos)
}

function draw() {
    setUniforms()
    cam.updatePosition()
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    iTime += 0.01;
}