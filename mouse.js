class Mouse {
    constructor() {
        this.pressed = false
        this.right_click = false
    }

    onClick(x, y, right_click) {
        this.pressed = true
        this.startX = x
        this.startY = y
        this.startPitch = cam.pitch
        this.startYaw = cam.yaw
    }

    onDrag(x, y) {
        const xOffset = x - this.startX
        const yOffset = y - this.startY
        const sensitivity = 0.2

        cam.yaw = this.startYaw + xOffset * sensitivity
        cam.pitch = this.startPitch - yOffset * sensitivity
    }

    onHold() {
    }

    onRelease() {
        this.pressed = false
    }
}

window.oncontextmenu = function(event) {
    event.preventDefault()
}

window.onmousedown = function(event) {
    mouse.onClick(event.clientX, event.clientY, event.which == 3)
}

window.onmousemove = function(event) {
    if(mouse.pressed) {
        mouse.onDrag(event.clientX, event.clientY)
    }
}

window.onmouseup = function(event) {
    mouse.onRelease(event.clientX, event.clientY)
}