// %it gives print statements about the direction and div i think, but this might not be the most informative so we should probably change it to the positions and then the x and y but i did not have as much time as i hoped so if you guys have time this weekend you can take a look at it. I will work further on this after the week

const outputDiv = document.getElementById('output');
const output = [];

class Particle {
    constructor() {
        this.pos = createVector(random(0, Scene.w), random(0, Scene.h));
        this.dir = p5.Vector.random2D().setMag(4); // Set magnitude to fixed speed
    }

    wrap() {
        if (this.pos.x < 0) this.pos.x += Scene.w;
        if (this.pos.y < 0) this.pos.y += Scene.h;
        if (this.pos.x > Scene.w) this.pos.x -= Scene.w;
        if (this.pos.y > Scene.h) this.pos.y -= Scene.h;
    }

    step() {
        const N = Scene.neighbours(this.pos);
        let avg_sin = 0, avg_cos = 0;
        const avg_p = createVector(0, 0);
        const avg_d = createVector(0, 0);

        for (let n of N) {
            avg_p.add(n.pos);
            if (n !== this) {
                let away = p5.Vector.sub(this.pos, n.pos);
                away.div(away.magSq());
                avg_d.add(away);
            }
            avg_sin += Math.sin(n.dir.heading()) / N.length;
            avg_cos += Math.cos(n.dir.heading()) / N.length;
        }

        avg_p.div(N.length);
        avg_d.div(N.length);
        let avg_angle = Math.atan2(avg_cos, avg_sin);
        avg_angle += Math.random() * 0.5 - 0.25;
        this.dir = p5.Vector.fromAngle(avg_angle);
        const cohesion = p5.Vector.sub(avg_p, this.pos);
        cohesion.div(100);
        this.dir.add(cohesion);
        avg_d.mult(20);
        this.dir.add(avg_d);
        this.dir.mult(4);
        this.pos.add(this.dir);
        this.wrap();
        output.push(`(${this.pos.x.toFixed(2)}, ${this.pos.y.toFixed(2)})`);
    }
}

const Scene = {
    w: 600,
    h: 600,
    swarm: [],
    neighbours(x) {
        let r = []
        for (let p of this.swarm) {
            if (dist(p.pos.x, p.pos.y, x.x, x.y) <= 100) {
                r.push(p)
            }
        }
        return r
    }
};

function setup() {
    for (let _ of Array(200)) {
        Scene.swarm.push(new Particle());
    }
}

function draw() {
    for (let p of Scene.swarm) {
        p.step();
    }
    outputDiv.innerText = output.join('\n');
}

// Initialize and run
setup();
draw();
