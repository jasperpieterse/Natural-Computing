const outputDiv = document.getElementById('output');
const output = []; // Store output data

// PARAMETERS

//Boid simulation parameters
let speed = 0.3;
let numBoids = 15; // Number of boids in the simulation
let fieldSize = 165; // Size of the simulation field
let numSteps = 300; // Number of simulation steps

// Boid behavior parameters
let cohesionStrength = 100;
let separationStrength = 20;
let alignmentStrength = 1; 
let interactionRadius = 28;
let exclusionRadius = 6;

// ABC parameters
const numIterations = 10; // Number of ABC iterations
const acceptedPopulationSize = 20; // Number of accepted parameters to store
const distanceThreshold = 0.01; // Adjust based on your distance function

// ABC priors
let cohesionPrior = [40, 160]; // Range for cohesion parameter
let alignmentPrior = [0.1, 3];  // Range for alignment parameter
let separationPrior = [10, 80];   // Range for separation parameter

// ABC perturbation standard deviations 
let stdDev_coh = 5; // Standard deviation for cohesion
let stdDev_sep = 2; // Standard deviation for separation
let stdDev_align = 0.1; // Standard deviation for alignment

// Initialize arrays for simulation
let orderParameter = []; // Array to store order parameter over time
let neighborDistances = []; // Array to store nearest-neighbor distances over time
let acceptedParameters = []; // Store accepted parameters
let timeStep = 0;

// Initialize ABC
let isAbcRunning = false; // Flag to indicate if ABC is active
let currentIteration = 0; // Current ABC iteration counter
let bestDistance = Infinity; // Initialize with a large distance


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

    perturbedParameters[0] += randomGaussian(0, stdDev_coh); // Perturb cohesion
    perturbedParameters[1] += randomGaussian(0, stdDev_align); // Perturb alignment
    perturbedParameters[2] += randomGaussian(0, stdDev_sep); // Perturb separation

    return perturbedParameters;
}

function runABC() {
    isAbcRunning = true; // Start ABC

    let currentPriors = [cohesionPrior, alignmentPrior, separationPrior];
    console.log("Starting ABC with priors:", currentPriors);
    let acceptedParameters = []; // Store accepted parameters outside of the fucking loop because Javascript is fucking worthless

    // Generate candidate parameters
    for (let iteration = 0; iteration < numIterations; iteration++) {
        currentIteration = iteration;  // Update current iteration flag for visualization
        acceptedParameters = []; // Reset for each iteration

        attempts = 0 
        while (acceptedParameters.length < acceptedPopulationSize) {
            attempts++ // increment attempt counter
            let sampledFromPriors = sampleFromPriors(currentPriors);  // Sample some parameters from our prior
            let sampledParameters = perturbParameters(sampledFromPriors)  // Perturb them using Gaussian noise
            runBoidSimulation(sampledParameters); 
            let orderParameter = calculateOrderParameter();
            let distance = calculateDistance(1, orderParameter); // Target = 1

            // console.log("Parameters:", sampledParameters);
            // console.log("Distance:", distance);

            // Update best distance if needed
            if (distance < bestDistance) {
                bestDistance = distance;
            }

            if (distance <= distanceThreshold) {
                acceptedParameters.push(sampledParameters);
                // console.log("Accepted Parameters:", sampledParameters);
            }
        } 

        // Update priors based on for next iteration
        currentPriors = updatePriors(acceptedParameters); 
        console.log("Finished ABC Iteration:", iteration);
        console.log("within this amount of attempts:", attempts)
        console.log("Updated Priors:", currentPriors);
    }
  
    // Analyze results in accepted parameters, potentially save them
    console.log("Final Accepted Parameters:", acceptedParameters);
    isAbcRunning = false; // ABC process finished

    
  }

  
function sampleFromPriors(priors) {
    let sampledParameters = [];
  
    // Sample from each prior distribution
    for (let prior of priors) {
      let min = prior[0];
      let max = prior[1];
      let sampledValue = random(min, max); 
      sampledParameters.push(sampledValue);
    }
  
    return sampledParameters;
  }
  

function updatePriors(acceptedParameters) {
    let newPriors = [];
  
    // Update priors for each parameter
    for (let i = 0; i < acceptedParameters[0].length; i++) {
      let parameterValues = acceptedParameters.map(params => params[i]);
  
      // Calculate new mean and standard deviation
      let newMean = calculateMean(parameterValues);
      let newStdDev = calculateStandardDeviation(parameterValues);
  
      // Avoid standard deviations that are too small
      if (newStdDev < 0.1) { 
        newStdDev = 0.1; 
      }
  
      newPriors.push([newMean - 3 * newStdDev, newMean + 3* newStdDev]); 
    }
  
    return newPriors;
  }
function calculateMean(values) {
    const sum = values.reduce((accumulator, current) => accumulator + current, 0);
    return sum / values.length;
  }
  
  function calculateStandardDeviation(values) {
    const mean = calculateMean(values);
    const squaredDeviations = values.map(value => (value - mean) ** 2); 
    const variance = squaredDeviations.reduce((sum, dev) => sum + dev, 0) / values.length;
  
    return Math.sqrt(variance);
  }
  
  
function runBoidSimulation(parameters) {
    // Reset scene and swarm for a new simulation run
    initScene(); 
    initSwarm(); 
  
    // Unpack parameters
    const [cohesionStrength, alignmentStrength, separationStrength] = parameters; 
  
    // Main simulation loop 
    for (let i = 0; i < numSteps; i++) {
      for (let p of Scene.swarm) {
        // Update Boid parameters with provided values
        p.cohesionStrength = cohesionStrength;  // Update if you have a matching property in "Particle"
        p.alignmentStrength = alignmentStrength;
        p.separationStrength = separationStrength;
  
        p.step();
        p.wrap();  
        p.draw();
      }
    }
  
    // Calculate order parameter at the end of the simulation
    return calculateOrderParameter();
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


function setup() {
    createCanvas(fieldSize, fieldSize); 
    initScene(); // Initialize your scene
    initSwarm(); // Initialize the initial swarm

    // Create a button for triggering ABC
    let abcButton = createButton('Run ABC');
    abcButton.mousePressed(runABC); 
}

function draw() {
    background(220); // Clear the background

    // Calculate metrics each frame, even if simulation is not running
    const currentOrderParameter = calculateOrderParameter(); 
    const currentNeighborDistances = calculateNearestNeighborDistances();

    displaySimulation(); // Update and display the boids
    displayParameters(currentOrderParameter, currentNeighborDistances); 

    
    // Visualization of ABC progress
    if (isAbcRunning) { // Display ABC progress if ABC is running
        textSize(14);

        // Textual display
        text(`ABC Iteration: ${currentIteration}`, 10, height - 60);
        text(`Best Distance: ${bestDistance.toFixed(2)}`, 10, height - 40);

        // Simple chart for cohesion parameter
        stroke(0, 0, 255); // Blue for cohesion
        line(50, height - 30, 50 + currentIteration * 5, height - 30 - acceptedParameters[currentIteration][0] * 20);
    }
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
    textSize(16);
    text(`Order Parameter: ${orderParam.toFixed(2)}`, 10, 20);
    text(`Avg. Neighbor Distance: ${neighborDist.toFixed(2)}`, 10, 40);
    text(`Time Step: ${timeStep}`, 10, 60);
}


