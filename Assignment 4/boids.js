const outputDiv = document.getElementById('output');
const output = [];

//Define if want output visually (false) or quantitatively (true)
let textoutput= false;
//Define speed
let speed = 4;

//Calculations
let orderParameter = []; // Array to store order parameter over time
let neighborDistances = []; // Array to store nearest-neighbor distances over time
let timeStep = 0;

// Calculate order parameter
function calculateOrderParameter() {
    let sumOfVelocities = createVector(0, 0);
	//Sum velocities of all boids (normalized so interval is [0,1]) 
    for (let p of Scene.swarm) {
		let normalizedVelocity = p.dir.copy().div(p.dir.mag());
        sumOfVelocities.add(normalizedVelocity);
    }
	// Normalize so non-negative
    let order = sumOfVelocities.mag() / Scene.swarm.length; 
    return order;
}

// Calculate nearest-neighbor distances
function calculateNearestNeighborDistances() {
    let distances = [];
    for (let p of Scene.swarm) {
		let minDistance = Infinity;

        for (let other of Scene.swarm) {
            if (p !== other) {
                // Calculate distance between p1 and p2
                let distance = dist(p.pos.x, p.pos.y, other.pos.x, other.pos.y);

                // Update minimum distance if distance to p2 is smaller
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }

        // Store minimum distance for each Boid
        distances.push(minDistance);
    }
	// Calculate the total sum of distances
	let sumOfDistances = distances.reduce((accumulator, currentValue) => accumulator + currentValue, 0);

	// Calculate the average distance
	let averageDistance = sumOfDistances / distances.length;
    return averageDistance;
}


class Particle {
	constructor(v){
		this.pos = createVector( random(0,Scene.w), random(0,Scene.h) )
		this.dir = p5.Vector.random2D().setMag(v) // Set magnitude to fixed speed
		this.v = v;
	}

    //make sure bois stay in field (wrap means Boids that move beyond one edge 
    // of the simulation field reappear on the opposite side)
	wrap(){
		if( this.pos.x < 0 ) this.pos.x += Scene.w
		if( this.pos.y < 0 ) this.pos.y += Scene.h
		if( this.pos.x > Scene.w ) this.pos.x -= Scene.w
		if( this.pos.y > Scene.h ) this.pos.y -= Scene.h
	}

	draw(){
		fill( 0 )
		ellipse( this.pos.x, this.pos.y, 5, 5 )
	}

	step(){
		// Parameters
		let cohesionStrength = 100;
		let separationStrength = 20;
		let alignmentStrength = 1; // Add this
		let interactionRadius = 100; // Add this

    //List of positions of all neighbours (within radius of interactionRadius)
    let N = Scene.neighbours( this.pos, interactionRadius ), 
        //together average dir
        avg_sin = 0, avg_cos = 0,
        //Stores avg position and avg direction of neighbours
        avg_p = createVector( 0, 0 ),
        avg_d = createVector( 0, 0 )
    for( let n of N  ){ //each neighbour
        avg_p.add( n.pos )
        if( n != this ){
            //make vector for going to n neighbour if != self
            let away = p5.Vector.sub( this.pos, n.pos )
            //Scale vector by dividing by sq of magnitude
            away.div( away.magSq() ) 
            avg_d.add( away )
        }
        avg_sin += Math.sin( n.dir.heading() ) / N.length
        avg_cos += Math.cos( n.dir.heading() ) / N.length
		
    	}

		// Alignment behavior
		let alignmentVector = createVector(0, 0);
		for (let n of N) { // For each neighbor within the interactionRadius
		  alignmentVector.add(n.dir); // Sum up the direction vectors of neighbors
		}
		alignmentVector.div(N.length); // Average the neighboring directions
		alignmentVector.setMag(alignmentStrength); // Scale by the alignmentStrength parameter
		this.dir.add(alignmentVector); // Add alignment to the direction calculation

		// Exclusion behavior
		let exclusionRadius = 20; // Adjust the radius value as needed
		let exclusionVector = createVector(0, 0);
	
		for (let n of N) {
		let distance = p5.Vector.dist(this.pos, n.pos);
		if (distance > 0 && distance < exclusionRadius) { // Note the 'distance > 0' to avoid self-interaction
			// Create a vector pointing away from the neighbor within the exclusion zone
			let repulsion = p5.Vector.sub(this.pos, n.pos); 
			repulsion.normalize(); // Ensure unit vector directionality
			repulsion.div(distance); // Weight repulsion by inverse distance
			exclusionVector.add(repulsion);
		}
		}
	
		if (exclusionVector.mag() > 0) { // Apply only if exclusion force exists
		exclusionVector.setMag(separationStrength); // Reuse your existing separationStrength
		this.dir.add(exclusionVector); 
		}
		//calc avg position and direction with a
		avg_p.div( N.length ); avg_d.div( N.length )
		let avg_angle = Math.atan2( avg_cos, avg_sin )
		//add random perturbaration for variability
		avg_angle += Math.random()*0.5 - 0.25
		//update boids dir
		this.dir = p5.Vector.fromAngle( avg_angle )
		//calc cohesion vector (where self should go for average position)
		let cohesion = p5.Vector.sub( avg_p, this.pos )
		cohesion.div( cohesionStrength )  //scale (play with cohesion)
		this.dir.add( cohesion )
		//scale average away vector (separation) to avoid collisions (play with separation)
		avg_d.mult( separationStrength )
		this.dir.add( avg_d )
		//Scale the final direction (x4) vector to control the boid's speed
		this.dir.mult( this.v )
		this.pos.add( this.dir )
		// make boid stay inside of field
		this.wrap()
		output.push(`(${this.pos.x.toFixed(2)}, ${this.pos.y.toFixed(2)})`);
	}
}


let Scene = {
	w : 600, h : 600, swarm : [],
	neighbours( x ){
		let r = []
		for( let p of this.swarm ){
			if( dist( p.pos.x, p.pos.y, x.x, x.y ) <= 100 ){
				r.push( p )
			}
		}
		return r
	}
}

//Automatically called in .js library
function setup(){
	if(!textoutput){
		createCanvas( Scene.w, Scene.h )
	}
	for( let _ of Array(200) ){
		Scene.swarm.push( new Particle(speed) )
	}
}

//Automatically called in .js library
function draw(){
	
	//  Calculate order parameter and store it over time
	let currentOrder = calculateOrderParameter();
	orderParameter.push(currentOrder);

	// Calculate nearest-neighbor distances and store them over time
	let currentDistances = calculateNearestNeighborDistances();
	neighborDistances.push(currentDistances);

	if (timeStep >= 300) {
        noLoop(); // This is a p5.js function that stops the draw() function from looping

	}

	if(!textoutput){
		background( 220 );
	}
	for( let p of Scene.swarm ){
		p.step()
		if(!textoutput){
			p.draw()
		}
	    // Display time step 
		textSize(20); 
		fill(0); // Black text for visibility
		text(`Time Step: ${timeStep}`, 20, 30); // Position in the top-left

		// Progress bar (assuming your canvas is Scene.w wide)
		let barWidth = map(timeStep, 0, 300, 0, Scene.w); // Adjust '300' to your desired max time
		fill(0, 128, 255); // Blue color
		rect(0, Scene.h - 20, barWidth, 20); // Position at the bottom
	}
	if (textoutput){
		outputDiv.innerText = output.join('\n');
		outputDiv.innerText = orderParameter.join('\n');

	}
	timeStep++; // Increment time step counter
}


