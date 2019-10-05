const R = require('ramda')
const p5 = require('p5')

//GLOBAL STATE

let canvas;
let state = {}
let stateHistory = []
let Global = {}

function setup() {
    canvas = p5.createCanvas(windowWidth, windowHeight)
    
    Global = {
        //initial state
        envThreshold: 2*(windowWidth/3.0),
        walkerNum: 50,
        driverNum: 3,
        walkerMass: 10.0,
        driverMass: 100.0,
        walkerDistr: _ => createVector(random(windowWidth), random(windowHeight)),
        driverDistr: _ => createVector(random(Global.envThreshold), random(windowHeight)),

        //force constants
        GConst: 500.0,
        liqRepConst: 200.0,
        viscosityConst: 50.0,

        //rendering settings
        walkerRadius: 5,
        driverRadius: 25,
        airBackgroundColor: color('#8c97b0').setAlpha(150),
        liqBackgroundColor: color('#a97db0').setAlpha(150),
        walkerColor: color('#8ba6eb').setAlpha(150),
        driverColor: color('#ff7300').setAlpha(150),
    }

    console.log(Global.driverMass) //DEBUGGING
    state = createState(Global.walkerNum, Global.driverNum)(state)
}

//LENSES

const walkerLens = R.lensProp('walkers')
const driverLens = R.lensProp('drivers')
const posLens = R.lensProp('position')
const velLens = R.lensProp('velocity')
const massLens = R.lensProp('mass')

const setPos = R.set(posLens)
const setVel = R.set(velLens)
const setVel0 = R.set(velLens)(0)
const setMass = R.set(massLens)

//MODULES

const Environ = (function() { //simulates environmental constraints such as materials and collisions

    //topPredicate: walker --> boolean (returns true if walker is at the top edge)
    const topPredicate = walker => R.view(posLens)(walker).y <= Global.walkerRadius
    const rightPredicate = walker => R.view(posLens)(walker).x + Global.walkerRadius >= windowWidth
    const bottomPredicate = walker => R.view(posLens)(walker).y + Global.walkerRadius >= windowHeight
    const leftPredicate = walker => R.view(posLens)(walker).x <= Global.walkerRadius

    //xReflect: object --> object (with reflected velocity if at top or bottom edge, else return unchanged)
    const xReflect = R.ifElse(R.anyPass([topPredicate, bottomPredicate]),
        object => R.set(velLens)(createVector(R.view(velLens)(object).x, (R.view(velLens)(object).y)*(-1)))(object),
        R.identity)
    const yReflect = R.ifElse(R.anyPass([rightPredicate, leftPredicate]),
        object => R.set(velLens)(createVector((R.view(velLens)(object).x)*(-1), R.view(velLens)(object).y))(object),
        R.identity)

    //topTruncate: object --> object (Truncates velocity vector component if object is at edge, else returned unchanged)
    const topTruncate = R.ifElse(R.allPass([topPredicate, object => R.view(velLens)(object).y < 0]),
        object => R.set(velLens)(createVector(R.view(velLens)(object).x, 0))(object),
        R.identity)
    const rightTruncate = R.ifElse(R.allPass([rightPredicate, object => R.view(velLens)(object).x > 0]),
        object => R.set(velLens)(createVector(0, R.view(velLens)(object).y))(object),
        R.identity)
    const bottomTruncate = R.ifElse(R.allPass([bottomPredicate, object => R.view(velLens)(object).y > 0]),
        object => R.set(velLens)(createVector(R.view(velLens)(object).x, 0))(object),
        R.identity)
    const leftTruncate = R.ifElse(R.allPass([leftPredicate, object => R.view(velLens)(object).x < 0]),
        object => R.set(velLens)(createVector(0, R.view(velLens)(object).y))(object),
        R.identity)

    //beyondThreshold: object --> boolean
    const beyondThreshold = object => (R.view(posLens)(object)).x >= Global.envThreshold

    //edgeCollision: object --> object (simulates bouncing by changing velocity if object is at edge)
    const edgeCollision = R.compose(yReflect, xReflect)

    //edgeTruncate: object --> object (modifies velocity if it points into the wall)
    const edgeTruncate = R.compose(leftTruncate, bottomTruncate, rightTruncate, topTruncate)

    return {
        beyondThreshold: beyondThreshold,
        edgeCollision: edgeCollision,
        edgeTruncate: edgeTruncate,
    }
})()

const Forces = (function() { //simulates external natural forces

    //gRepulsion: object --> object --> Vector (acceleration) calculates repulsion caused by second object on first object
    const gRepulsion = obj1 => obj2 => p5.Vector.mult(p5.Vector.sub(R.view(posLens)(obj1), R.view(posLens)(obj2)).normalize(),
        (Global.GConst * R.view(massLens)(obj2))/(p5.Vector.dist(R.view(posLens)(obj1), R.view(posLens)(obj2)) ** 2))

    //liquidRepulsion: object --> Vector (acceleration)
    const liquidRepulsion = object => p5.Vector.mult(createVector(-1, 0),
        Global.liqRepConst*((R.view(posLens)(object).x-Global.envThreshold)**0.5))

    //fluidResistance: object --> Vector (acceleration)
    const fluidResistance = object => p5.Vector.mult(p5.Vector.mult(R.view(velLens)(object), -1),
        (R.view(velLens)(object).mag())*(Global.viscosityConst/R.view(massLens)(object)))

    //applyForce: object --> Vector (acceleration) --> object (returns object with new velocity)
    const applyForce = object => acc => setVel(p5.Vector.add(acc, R.view(velLens)(object)))(object)

    return {
        gRepulsion: gRepulsion,
        liquidRepulsion: liquidRepulsion,
        fluidResistance: fluidResistance,
        applyForce: applyForce,
    }
})()

const Steering = (function() {//simulates autonomous behaviors

})()

//HELPER FUNCTIONS - STATE CREATION AND MUTATION

//factory function
const walkerFactory = (pos, mass) => R.compose(setMass(mass), setVel0, setPos(pos))({})

//factory function
const driverFactory = (pos, mass) => R.compose(setMass(mass), setVel0, setPos(pos))({})

//createN: return list of objects
const createN = R.ifElse(
    (N, _) => N===0,
    (N, factory, posDistr, massDistr, arr) => arr,
    function(N, factory, posDistr, massDistr, arr) {
        console.log(posDistr) //DEBUGGING
        return createN(N-1, factory, posDistr, massDistr, R.append(factory(posDistr(N), massDistr(N)))(arr))
    })

//createState: (num of walkers, num of drivers) --> empty object --> init state
const createState = (wN, dN) => R.compose(R.set(driverLens)(createN(dN, driverFactory, Global.driverDistr, _ => Global.driverMass, [])), 
                                    R.set(walkerLens)(createN(wN, walkerFactory, Global.walkerDistr, _ => Global.walkerMass, [])))

//moveObject: object --> object (applies velocity to the position)
const moveObject = object => R.over(posLens)((pos) => p5.Vector.add(pos, R.view(velLens)(object)))(object)

//walkerAirTransform: walker --> walker (with new velocity but unchanged position)
const walkerAirTransform = function(walker) {
    const totalAcc = R.reduce((a, driver) => p5.Vector.add(Forces.gRepulsion(walker)(driver), a))(createVector(0, 0))(R.view(driverLens)(state))
    return Forces.applyForce(walker)(totalAcc)
}

//walkerLiquidTransform: walker --> walker (with new velocity but unchanged position)
const walkerLiquidTransform = function(walker) {
    const accGRepulse = R.reduce((a, driver) => p5.Vector.add(Forces.gRepulsion(walker)(driver), a))(createVector(0, 0))(R.view(driverLens)(state))
    const totalAcc = p5.Vector.add(p5.Vector.add(Forces.liquidRepulsion(walker), Forces.fluidResistance(walker)), accGRepulse)
    return Forces.applyForce(walker)(totalAcc)
}

//walkerTransform: walker --> walker (collision detection, force calculation, application, environmental calculation, motion)
const walkerTransform = R.compose(moveObject, Environ.edgeTruncate, R.ifElse(
    Environ.beyondThreshold,
    walkerLiquidTransform,
    walkerAirTransform
    ), Environ.edgeCollision)

//driverTransform: driver --> driver
const driverTransform = R.identity

//newState: state --> state
const newState = R.compose(R.over(driverLens)(R.map(driverTransform)), R.over(walkerLens)(R.map(walkerTransform)))

//ENTRY POINT AND IMPURE METHODS

//drawEnv void --> side effects (renders environment/background to screen using rendering and environment info in Global)
const drawEnv = function() {
    noStroke()
    fill(Global.airBackgroundColor)
    rect(0, 0, Global.envThreshold, windowHeight)
    fill(Global.liqBackgroundColor)
    rect(Global.envThreshold, 0, (windowWidth-Global.envThreshold), windowHeight)
}

//drawState state --> side effects (renders current state info to screen using rendering settings in Global, and object rendering tags)
const drawState = function(state) {
    const drawWalker = function(walker) {
        noStroke()
        fill(Global.walkerColor)
        circle(R.view(posLens)(walker).x, R.view(posLens)(walker).y, Global.walkerRadius)
    }
    const drawDriver = function(driver) {
        noStroke()
        fill(Global.driverColor)
        circle(R.view(posLens)(driver).x, R.view(posLens)(driver).y, Global.driverRadius)
    }
    R.forEach(drawDriver)(R.view(driverLens)(state))
    R.forEach(drawWalker)(R.view(walkerLens)(state))
}

function draw() {
    clear()
    drawEnv()
    drawState(state)
    stateHistory = R.append(state)(stateHistory)
    state = newState(state)
}