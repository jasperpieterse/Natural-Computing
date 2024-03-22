const outputDiv = document.getElementById('output');
const output = []; // Store output data

// PARAMETERS

//Boid simulation parameters
let speed = 4;
let numBoids = 50; // Number of boids in the simulation
let fieldSize = 600; // Size of the simulation field
let numSteps = 300; // Number of simulation steps

// ABC parameters
const numIterations = 10; // Number of ABC iterations
const acceptedPopulationSize = 20; // Number of accepted parameters to store
const distanceThreshold = 0.1; // Adjust based on your distance function

// ABC priors
let cohesionPrior = [0.5, 2.0]; // Example prior for cohesion (min, max)
let alignmentPrior = [0.1, 1.5];  // Example prior for alignment
let separationPrior = [10, 50];   // Example prior for separation

// Initialize arrays for simulation
let orderParameter = []; // Array to store order parameter over time
let neighborDistances = []; // Array to store nearest-neighbor distances over time
let acceptedParameters = []; // Store accepted parameters
let timeStep = 0;

let Scene = {
    w: fieldSize,    // Width of the simulation area
    h: fieldSize,   // Height of the simulation area
    swarm: [],     // An array to store 'Boid' or particle objects

    neighbours( x ) {  // Function to find neighbors of a given point/particle
        let r = [];    // Initialize an empty array to store the neighbors

        for (let p of this.swarm) {  // Iterate over each particle 'p' in the swarm 
            if (dist(p.pos.x, p.pos.y, x.x, x.y) <= 100) {  // Calculate distance between 'p' and the input point 'x'
                r.push(p);   // If distance is less than or equal to 100, add 'p' to the neighbors array
            }
        }
        return r;      // Return the array of neighboring particles 
    }
}

// BOIDS SIMULATION
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


// EVACUATION SIMULATION

// Function to calculate order parameter
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

// Function to alculate nearest-neighbor distances
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


function calculateDistance(targetOrderParameter, simulatedOrderParameter) {
    return Math.abs(targetOrderParameter - simulatedOrderParameter);
  }

function perturbParameters(parameters) {
let perturbedParameters = parameters.slice();  // Make a copy

// Adjust standard deviation as needed
const stdDev = 5; 

perturbedParameters[0] += randomGaussian(0, stdDev); // Perturb cohesion
perturbedParameters[1] += randomGaussian(0, stdDev); // Perturb alignment
perturbedParameters[2] += randomGaussian(0, stdDev); // Perturb separation

return perturbedParameters;
}

function runABC() {
    let currentPriors = [cohesionPrior, alignmentPrior, separationPrior];
  
    for (let iteration = 0; iteration < numIterations; iteration++) {
      let candidateParameters = [];
      let acceptedParameters = [];
  
      // Generate candidate parameters
      for (let i = 0; i < acceptedPopulationSize * 5; i++) { // Generate  ample candidates
        let sampledParameters = sampleFromPriors(currentPriors); 
        candidateParameters.push(sampledParameters);
      }
  
      // Evaluate candidates and accept parameters
      for (let parameters of candidateParameters) {
        runBoidSimulation(parameters); // Update your simulation function
        let orderParameter = calculateOrderParameter();
        let distance = calculateDistance(1, orderParameter); // Target = 1
  
        if (distance <= distanceThreshold) {
          acceptedParameters.push(parameters);
        }
      }
  
      // Check if we have enough accepted parameters
      if (acceptedParameters.length >= acceptedPopulationSize) {
        updatePriors(acceptedParameters);
        break; // Stop if we have converged enough
      }
  
      // Update priors for next iteration
      currentPriors = updatePriors(acceptedParameters); 
    }
  
    // Analyze results in accepted parameters, potentially save them
    console.log("Final Accepted Parameters:", acceptedParameters);
  }
  


function initScene() {
    Scene = {
        w: fieldSize,    // Width of the simulation area
        h: fieldSize,   // Height of the simulation area
        swarm: [],     // An array to store 'Boid' or particle objects
    
        neighbours( x ) {  // Function to find neighbors of a given point/particle
            let r = [];    // Initialize an empty array to store the neighbors
    
            for (let p of this.swarm) {  // Iterate over each particle 'p' in the swarm 
                if (dist(p.pos.x, p.pos.y, x.x, x.y) <= 100) {  // Calculate distance between 'p' and the input point 'x'
                    r.push(p);   // If distance is less than or equal to 100, add 'p' to the neighbors array
                }
            }
            return r;      // Return the array of neighboring particles 
        }
    }
}

function initSwarm() {
     Scene.swarm = []; // Clear previous boids
     for (let i = 0; i < numBoids; i++) {
        Scene.swarm.push(new Particle(speed));
    }
}

let shouldUpdateSimulation = true; // Flag to trigger new simulation

function setup() {
    createCanvas(fieldSize, fieldSize); 
    initScene(); // Initialize your scene
    initSwarm(); // Initialize the initial swarm
}

function draw() {
    background(220); // Clear the background

    // if (shouldUpdateSimulation) { 
    //     RunSimulation();  // Trigger a new ABC simulation run
    //     shouldUpdateSimulation = true; 
    // }

    // Calculate metrics each frame, even if simulation is not running
    const currentOrderParameter = calculateOrderParameter(); 
    const currentNeighborDistances = calculateNearestNeighborDistances();

    displaySimulation(); // Update and display the boids
    displayParameters(currentOrderParameter, currentNeighborDistances); 
}

function displaySimulation() {
    for (let p of Scene.swarm) {
        p.step(); // Update boids if simulation is running
        p.wrap(); 
        p.draw();  
    }
    timeStep++; // Increment time step
}

function displayParameters(orderParam, neighborDist) {
    // Create text display area (adjust CSS as needed)
    textSize(16);
    text(`Order Parameter: ${orderParam.toFixed(2)}`, 10, 20);
    text(`Avg. Neighbor Distance: ${neighborDist.toFixed(2)}`, 10, 40);
    text(`Time Step: ${timeStep}`, 10, 60);
}


