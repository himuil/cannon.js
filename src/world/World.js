/* global performance */

module.exports = World;

var Shape = require('../shapes/Shape');
var Vec3 = require('../math/Vec3');
var Quaternion = require('../math/Quaternion');
var GSSolver = require('../solver/GSSolver');
var ContactEquation = require('../equations/ContactEquation');
var FrictionEquation = require('../equations/FrictionEquation');
var Narrowphase = require('./Narrowphase');
var EventTarget = require('../utils/EventTarget');
var ArrayCollisionMatrix = require('../collision/ArrayCollisionMatrix');
var ObjectCollisionMatrix = require('../collision/ObjectCollisionMatrix');
var OverlapKeeper = require('../collision/OverlapKeeper');
var Material = require('../material/Material');
var ContactMaterial = require('../material/ContactMaterial');
var Body = require('../objects/Body');
var TupleDictionary = require('../utils/TupleDictionary');
var RaycastResult = require('../collision/RaycastResult');
var AABB = require('../collision/AABB');
var Ray = require('../collision/Ray');
var NaiveBroadphase = require('../collision/NaiveBroadphase');

if (global) {
    global['doProfiling'] = false;
    global['DEBUG'] = true;
} else if (window) {
    window['doProfiling'] = false;
    window['DEBUG'] = true;
}

/**
 * The physics world
 * @class World
 * @constructor
 * @extends EventTarget
 * @param {object} [options]
 * @param {Vec3} [options.gravity]
 * @param {boolean} [options.allowSleep]
 * @param {Broadphase} [options.broadphase]
 * @param {Solver} [options.solver]
 * @param {boolean} [options.quatNormalizeFast]
 * @param {number} [options.quatNormalizeSkip]
 */
function World (options) {
    options = options || {};
    EventTarget.apply(this);

    /**
     * Currently / last used timestep. Is set to -1 if not available. This value is updated before each internal step, which means that it is "fresh" inside event callbacks.
     * @property {Number} dt
     */
    this.dt = -1;

    /**
     * Makes bodies go to sleep when they've been inactive
     * @property allowSleep
     * @type {Boolean}
     * @default false
     */
    this.allowSleep = !!options.allowSleep;

    /**
     * All the current contacts (instances of ContactEquation) in the world.
     * @property contacts
     * @type {Array}
     */
    this.contacts = [];
    this.frictionEquations = [];

    this.contactsDic = new TupleDictionary();
    this.oldContactsDic = new TupleDictionary();

    /**
     * How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
     * @property quatNormalizeSkip
     * @type {Number}
     * @default 0
     */
    this.quatNormalizeSkip = options.quatNormalizeSkip !== undefined ? options.quatNormalizeSkip : 0;

    /**
     * Set to true to use fast quaternion normalization. It is often enough accurate to use. If bodies tend to explode, set to false.
     * @property quatNormalizeFast
     * @type {Boolean}
     * @see Quaternion.normalizeFast
     * @see Quaternion.normalize
     * @default false
     */
    this.quatNormalizeFast = options.quatNormalizeFast !== undefined ? options.quatNormalizeFast : false;

    /**
     * The wall-clock time since simulation start
     * @property time
     * @type {Number}
     */
    this.time = 0.0;

    /**
     * Number of timesteps taken since start
     * @property stepnumber
     * @type {Number}
     */
    this.stepnumber = 0;

    /// Default and last timestep sizes
    this.default_dt = 1 / 60;

    this.nextId = 0;
    /**
     * @property gravity
     * @type {Vec3}
     */
    this.gravity = new Vec3();
    if (options.gravity) {
        this.gravity.copy(options.gravity);
    }

    /**
     * The broadphase algorithm to use. Default is NaiveBroadphase
     * @property broadphase
     * @type {Broadphase}
     */
    this.broadphase = options.broadphase !== undefined ? options.broadphase : new NaiveBroadphase();

    /**
     * @property bodies
     * @type {Array}
     */
    this.bodies = [];

    /**
     * The solver algorithm to use. Default is GSSolver
     * @property solver
     * @type {Solver}
     */
    this.solver = options.solver !== undefined ? options.solver : new GSSolver();

    /**
     * @property constraints
     * @type {Array}
     */
    this.constraints = [];

    /**
     * @property narrowphase
     * @type {Narrowphase}
     */
    this.narrowphase = new Narrowphase(this);

    /**
     * @property {ObjectCollisionMatrix} collisionMatrix
	 * @type {ObjectCollisionMatrix}
	 */
    this.collisionMatrix = new ObjectCollisionMatrix();

    this.triggerMatrix = new ObjectCollisionMatrix();

    this.shapeOverlapKeeper = new OverlapKeeper();

    this.shapeOverlapKeeperExit = new OverlapKeeper();

    /**
     * All added materials
     * @property materials
     * @type {Array}
     */
    this.materials = [];

    /**
     * @property contactmaterials
     * @type {Array}
     */
    this.contactmaterials = [];

    /**
     * Used to look up a ContactMaterial given two instances of Material.
     * @property {TupleDictionary} contactMaterialTable
     */
    this.contactMaterialTable = new TupleDictionary();

    this.defaultMaterial = new Material("default");

    /**
     * This contact material is used if no suitable contactmaterial is found for a contact.
     * @property defaultContactMaterial
     * @type {ContactMaterial}
     */
    this.defaultContactMaterial = new ContactMaterial(this.defaultMaterial, this.defaultMaterial, { friction: 0.3, restitution: 0.0 });

    /**
     * @property profile
     * @type {Object}
     */
    this.profile = {
        solve: 0,
        makeContactConstraints: 0,
        broadphase: 0,
        integrate: 0,
        narrowphase: 0,
    };

    /**
     * Time accumulator for interpolation. See http://gafferongames.com/game-physics/fix-your-timestep/
     * @property {Number} accumulator
     */
    this.accumulator = 0;

    /**
     * @property subsystems
     * @type {Array}
     */
    this.subsystems = [];

    /**
     * Dispatched after a body has been added to the world.
     * @event addBody
     * @param {Body} body The body that has been added to the world.
     */
    this.addBodyEvent = {
        type: "addBody",
        body: null
    };

    /**
     * Dispatched after a body has been removed from the world.
     * @event removeBody
     * @param {Body} body The body that has been removed from the world.
     */
    this.removeBodyEvent = {
        type: "removeBody",
        body: null
    };

    this.broadphase.setWorld(this);
}

World.idToBodyMap = {};

World.idToShapeMap = {};

World.prototype = new EventTarget();

// Temp stuff
// var tmpAABB1 = new AABB();
// var tmpArray1 = [];
var tmpRay = new Ray();

/**
 * Get the contact material between materials m1 and m2
 * @method getContactMaterial
 * @param {Material} m1
 * @param {Material} m2
 * @return {ContactMaterial} The contact material if it was found.
 */
World.prototype.getContactMaterial = function (m1, m2) {
    return this.contactMaterialTable.get(m1.id, m2.id); //this.contactmaterials[this.mats2cmat[i+j*this.materials.length]];
};

/**
 * Get number of objects in the world.
 * @method numObjects
 * @return {Number}
 * @deprecated
 */
World.prototype.numObjects = function () {
    return this.bodies.length;
};

/**
 * Store old collision state info
 * @method collisionMatrixTick
 */
World.prototype.collisionMatrixTick = function () {
    // this.shapeOverlapKeeper.tick();
    // this.shapeOverlapKeeperExit.tick();
};

/**
 * Add a rigid body to the simulation.
 * @method add
 * @param {Body} body
 * @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
 * @todo Adding an array of bodies should be possible. This would save some loops too
 * @deprecated Use .addBody instead
 */
World.prototype.add = World.prototype.addBody = function (body) {
    if (this.bodies.indexOf(body) !== -1) {
        return;
    }
    body.index = this.bodies.length;
    this.bodies.push(body);
    body.world = this;
    body.initPosition.copy(body.position);
    body.initVelocity.copy(body.velocity);
    body.timeLastSleepy = this.time;
    if (body instanceof Body) {
        body.initAngularVelocity.copy(body.angularVelocity);
        body.initQuaternion.copy(body.quaternion);
    }
    this.collisionMatrix.setNumObjects(this.bodies.length);
    this.addBodyEvent.body = body;
    World.idToBodyMap[body.id] = body;
    this.dispatchEvent(this.addBodyEvent);
};

/**
 * Add a constraint to the simulation.
 * @method addConstraint
 * @param {Constraint} c
 */
World.prototype.addConstraint = function (c) {
    this.constraints.push(c);
};

/**
 * Removes a constraint
 * @method removeConstraint
 * @param {Constraint} c
 */
World.prototype.removeConstraint = function (c) {
    var idx = this.constraints.indexOf(c);
    if (idx !== -1) {
        this.constraints.splice(idx, 1);
    }
};

/**
 * Raycast test
 * @method rayTest
 * @param {Vec3} from
 * @param {Vec3} to
 * @param {RaycastResult} result
 * @deprecated Use .raycastAll, .raycastClosest or .raycastAny instead.
 */
World.prototype.rayTest = function (from, to, result) {
    if (result instanceof RaycastResult) {
        // Do raycastclosest
        this.raycastClosest(from, to, {
            skipBackfaces: true
        }, result);
    } else {
        // Do raycastAll
        this.raycastAll(from, to, {
            skipBackfaces: true
        }, result);
    }
};

/**
 * Ray cast against all bodies. The provided callback will be executed for each hit with a RaycastResult as single argument.
 * @method raycastAll
 * @param  {Vec3} from
 * @param  {Vec3} to
 * @param  {Object} options
 * @param  {number} [options.collisionFilterMask=-1]
 * @param  {number} [options.collisionFilterGroup=-1]
 * @param  {boolean} [options.skipBackfaces=false]
 * @param  {boolean} [options.checkCollisionResponse=true]
 * @param  {Function} callback
 * @return {boolean} True if any body was hit.
 */
World.prototype.raycastAll = function (from, to, options, callback) {
    options.mode = Ray.ALL;
    options.from = from;
    options.to = to;
    options.callback = callback;
    return tmpRay.intersectWorld(this, options);
};

/**
 * Ray cast, and stop at the first result. Note that the order is random - but the method is fast.
 * @method raycastAny
 * @param  {Vec3} from
 * @param  {Vec3} to
 * @param  {Object} options
 * @param  {number} [options.collisionFilterMask=-1]
 * @param  {number} [options.collisionFilterGroup=-1]
 * @param  {boolean} [options.skipBackfaces=false]
 * @param  {boolean} [options.checkCollisionResponse=true]
 * @param  {RaycastResult} result
 * @return {boolean} True if any body was hit.
 */
World.prototype.raycastAny = function (from, to, options, result) {
    options.mode = Ray.ANY;
    options.from = from;
    options.to = to;
    options.result = result;
    return tmpRay.intersectWorld(this, options);
};

/**
 * Ray cast, and return information of the closest hit.
 * @method raycastClosest
 * @param  {Vec3} from
 * @param  {Vec3} to
 * @param  {Object} options
 * @param  {number} [options.collisionFilterMask=-1]
 * @param  {number} [options.collisionFilterGroup=-1]
 * @param  {boolean} [options.skipBackfaces=false]
 * @param  {boolean} [options.checkCollisionResponse=true]
 * @param  {RaycastResult} result
 * @return {boolean} True if any body was hit.
 */
World.prototype.raycastClosest = function (from, to, options, result) {
    options.mode = Ray.CLOSEST;
    options.from = from;
    options.to = to;
    options.result = result;
    return tmpRay.intersectWorld(this, options);
};

/**
 * Remove a rigid body from the simulation.
 * @method remove
 * @param {Body} body
 * @deprecated Use .removeBody instead
 */
World.prototype.remove = function (body) {
    body.world = null;
    var n = this.bodies.length - 1,
        bodies = this.bodies,
        idx = bodies.indexOf(body);
    if (idx !== -1) {
        bodies.splice(idx, 1); // Todo: should use a garbage free method

        // Recompute index
        for (var i = 0; i !== bodies.length; i++) {
            bodies[i].index = i;
        }

        this.collisionMatrix.setNumObjects(n);
        this.removeBodyEvent.body = body;
        delete World.idToBodyMap[body.id];
        this.dispatchEvent(this.removeBodyEvent);
    }
};

/**
 * Remove a rigid body from the simulation.
 * @method removeBody
 * @param {Body} body
 */
World.prototype.removeBody = World.prototype.remove;

World.prototype.getBodyById = function (id) {
    return World.idToBodyMap[id];
};

World.prototype.getShapeById = function (id) {
    return World.idToShapeMap[id];
};

/**
 * Adds a material to the World.
 * @method addMaterial
 * @param {Material} m
 * @todo Necessary?
 */
World.prototype.addMaterial = function (m) {
    this.materials.push(m);
};

/**
 * Adds a contact material to the World
 * @method addContactMaterial
 * @param {ContactMaterial} cmat
 */
World.prototype.addContactMaterial = function (cmat) {

    // Add contact material
    this.contactmaterials.push(cmat);

    // Add current contact material to the material table
    this.contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
};

// performance.now()
if (DEBUG) {
    if (typeof performance === 'undefined') {
        performance = {};
    }
    if (!performance.now) {
        var nowOffset = Date.now();
        if (performance.timing && performance.timing.navigationStart) {
            nowOffset = performance.timing.navigationStart;
        }
        performance.now = function () {
            return Date.now() - nowOffset;
        };
    }
}

// var step_tmp1 = new Vec3();

/**
 * Step the physics world forward in time.
 *
 * There are two modes. The simple mode is fixed timestepping without interpolation. In this case you only use the first argument. The second case uses interpolation. In that you also provide the time since the function was last used, as well as the maximum fixed timesteps to take.
 *
 * @method step
 * @param {Number} dt                       The fixed time step size to use.
 * @param {Number} [timeSinceLastCalled]    The time elapsed since the function was last called.
 * @param {Number} [maxSubSteps=10]         Maximum number of fixed steps to take per function call.
 *
 * @example
 *     // fixed timestepping without interpolation
 *     world.step(1/60);
 *
 * @see http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
 */
World.prototype.step = function (dt, timeSinceLastCalled, maxSubSteps) {
    maxSubSteps = maxSubSteps || 10;
    timeSinceLastCalled = timeSinceLastCalled || 0;
    World_step_oldContacts = this.contacts.slice();
    if (timeSinceLastCalled === 0) { // Fixed, simple stepping

        this.internalStep(dt);

        // Increment time
        this.time += dt;

    } else {

        this.accumulator += timeSinceLastCalled;
        var substeps = 0;
        while (this.accumulator >= dt && substeps < maxSubSteps) {
            // Do fixed steps to catch up
            this.internalStep(dt);
            this.accumulator -= dt;
            substeps++;
        }

        var t = (this.accumulator % dt) / dt;
        for (var j = 0; j !== this.bodies.length; j++) {
            var b = this.bodies[j];
            b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
            b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
            b.previousQuaternion.normalize();
        }
        this.time += timeSinceLastCalled;
    }


    var contacts = this.contacts;
    var i = this.contacts.length;
    while (i--) {
        // Current contact
        var c = contacts[i];
        // Get current collision indeces
        var si = c.si;
        var sj = c.sj;
        var item = this.contactsDic.get(si.id, sj.id);
        if (item == null) {
            item = this.contactsDic.set(si.id, sj.id, []);
        }
        item.push(c);
    }

    // trigger 
    this.emitTriggeredEvents();

    // collision
    var key;
    var data;
    // is collision enter or stay
    i = this.contactsDic.getLength();
    while (i--) {
        key = this.contactsDic.getKeyByIndex(i);
        data = this.contactsDic.getDataByKey(key);

        if (data == null)
            continue;

        var bi = data[0].bi;
        var bj = data[0].bj;
        var si = data[0].si;
        var sj = data[0].sj;

        if (bi.allowSleep &&
            bi.type === Body.DYNAMIC &&
            bi.sleepState === Body.SLEEPING &&
            bj.sleepState === Body.AWAKE &&
            bj.type !== Body.STATIC
        ) {
            var speedSquaredB = bj.velocity.norm2() + bj.angularVelocity.norm2();
            var speedLimitSquaredB = Math.pow(bj.sleepSpeedLimit, 2);
            if (speedSquaredB >= speedLimitSquaredB * 2) {
                // bi._wakeUpAfterNarrowphase = true;
                bi.wakeUp();
            }
        }

        if (bj.allowSleep &&
            bj.type === Body.DYNAMIC &&
            bj.sleepState === Body.SLEEPING &&
            bi.sleepState === Body.AWAKE &&
            bi.type !== Body.STATIC
        ) {
            var speedSquaredA = bi.velocity.norm2() + bi.angularVelocity.norm2();
            var speedLimitSquaredA = Math.pow(bi.sleepSpeedLimit, 2);
            if (speedSquaredA >= speedLimitSquaredA * 2) {
                // bj._wakeUpAfterNarrowphase = true;
                bj.wakeUp();
            }
        }

        // Now we know that i and j are in contact. Set collision matrix state		
        if (this.collisionMatrix.get(bi, bj)) {
            // collision stay
            World_step_collideEvent.event = 'onCollisionStay';

        } else {
            this.collisionMatrix.set(bi, bj, true);
            // collision enter
            World_step_collideEvent.event = 'onCollisionEnter';
        }

        World_step_collideEvent.contacts = data;

        World_step_collideEvent.body = sj.body;
        World_step_collideEvent.selfShape = si;
        World_step_collideEvent.otherShape = sj;
        si.body.dispatchEvent(World_step_collideEvent);

        World_step_collideEvent.body = si.body;
        World_step_collideEvent.selfShape = sj;
        World_step_collideEvent.otherShape = si;
        sj.body.dispatchEvent(World_step_collideEvent);
    }
    var oldcontacts = World_step_oldContacts;
    for (i = oldcontacts.length; i--;) {
        // Current contact
        var c = oldcontacts[i];

        // Get current collision indeces
        var si = c.si;
        var sj = c.sj;
        if (this.oldContactsDic.get(si.id, sj.id) == null) {
            this.oldContactsDic.set(si.id, sj.id, c);
        }
    }

    this.shapeOverlapKeeper.reset();
    this.shapeOverlapKeeperExit.tick();

    // is collision exit
    i = this.oldContactsDic.getLength();
    while (i--) {
        key = this.oldContactsDic.getKeyByIndex(i);
        if (this.contactsDic.getDataByKey(key) == null) {
            data = this.oldContactsDic.getDataByKey(key);
            var bi = data.bi;
            var bj = data.bj;
            var si = data.si;
            var sj = data.sj;
            if (this.collisionMatrix.get(bi, bj)) {
                if (!bi.isSleeping() || !bj.isSleeping()) {
                    this.collisionMatrix.set(bi, bj, false);
                    // collision exit
                    World_step_collideEvent.event = 'onCollisionExit';
                    World_step_collideEvent.body = sj.body;
                    World_step_collideEvent.selfShape = si;
                    World_step_collideEvent.otherShape = sj;
                    World_step_collideEvent.contacts.length = 0;
                    World_step_collideEvent.contacts.push(data);
                    si.body.dispatchEvent(World_step_collideEvent);

                    World_step_collideEvent.body = si.body;
                    World_step_collideEvent.selfShape = sj;
                    World_step_collideEvent.otherShape = si;
                    sj.body.dispatchEvent(World_step_collideEvent);
                } else {
                    // not exit, due to sleeping
                }
            }
        }
    }

    this.contactsDic.reset();
    this.oldContactsDic.reset();
};

/**
 * Dispatched after the world has stepped forward in time.
 * @event postStep
 */
// World_step_postStepEvent = {type:"postStep"}, // Reusable event objects to save memory
/**
 * Dispatched before the world steps forward in time.
 * @event preStep
 */
// World_step_preStepEvent = {type:"preStep"},
var
    World_step_collideEvent = {
        type: "collide",
        event: '',
        body: null,
        selfShape: null,
        otherShape: null,
        contacts: null
    },
    World_step_oldContacts = [], // Pools for unused objects
    World_step_frictionEquationPool = [],
    World_step_p1 = [], // Reusable arrays for collision pairs
    World_step_p2 = [];
// World_step_gvec = new Vec3(), // Temporary vectors and quats
// World_step_vi = new Vec3(),
// World_step_vj = new Vec3(),
// World_step_wi = new Vec3(),
// World_step_wj = new Vec3(),
// World_step_t1 = new Vec3(),
// World_step_t2 = new Vec3(),
// World_step_rixn = new Vec3(),
// World_step_rjxn = new Vec3(),
// World_step_step_q = new Quaternion(),
// World_step_step_w = new Quaternion(),
// World_step_step_wq = new Quaternion(),
// invI_tau_dt = new Vec3()

World.prototype.internalStep = function (dt) {
    this.dt = dt;

    var world = this,
        that = this,
        contacts = this.contacts,
        p1 = World_step_p1,
        p2 = World_step_p2,
        N = this.numObjects(),
        bodies = this.bodies,
        solver = this.solver,
        gravity = this.gravity,
        profile = this.profile,
        DYNAMIC = Body.DYNAMIC,
        profilingStart,
        constraints = this.constraints,
        frictionEquationPool = World_step_frictionEquationPool,
        // gnorm = gravity.norm(),
        gx = gravity.x,
        gy = gravity.y,
        gz = gravity.z,
        i = 0;

    if (doProfiling) {
        profilingStart = performance.now();
    }

    // Add gravity to all objects
    for (i = 0; i !== N; i++) {
        var bi = bodies[i];
        if (bi.useGravity && bi.type === DYNAMIC) { // Only for dynamic bodies
            var f = bi.force, m = bi.mass;
            f.x += m * gx;
            f.y += m * gy;
            f.z += m * gz;
        }
    }

    // Update subsystems
    for (var i = 0, Nsubsystems = this.subsystems.length; i !== Nsubsystems; i++) {
        this.subsystems[i].update();
    }

    // Collision detection
    if (doProfiling) { profilingStart = performance.now(); }
    p1.length = 0; // Clean up pair arrays from last step
    p2.length = 0;
    this.broadphase.collisionPairs(this, p1, p2);
    if (doProfiling) { profile.broadphase = performance.now() - profilingStart; }

    // Remove constrained pairs with collideConnected == false
    var Nconstraints = constraints.length;
    for (i = 0; i !== Nconstraints; i++) {
        var c = constraints[i];
        if (!c.collideConnected) {
            for (var j = p1.length - 1; j >= 0; j -= 1) {
                if ((c.bodyA === p1[j] && c.bodyB === p2[j]) ||
                    (c.bodyB === p1[j] && c.bodyA === p2[j])) {
                    p1.splice(j, 1);
                    p2.splice(j, 1);
                }
            }
        }
    }

    // Generate contacts
    if (doProfiling) { profilingStart = performance.now(); }

    contacts.length = 0;

    // Transfer FrictionEquation from current list to the pool for reuse
    var NoldFrictionEquations = this.frictionEquations.length;
    for (i = 0; i !== NoldFrictionEquations; i++) {
        frictionEquationPool.push(this.frictionEquations[i]);
    }
    this.frictionEquations.length = 0;

    this.narrowphase.getContacts(
        p1,
        p2,
        this,
        contacts,
        null,
        this.frictionEquations,
        frictionEquationPool
    );

    if (doProfiling) {
        profile.narrowphase = performance.now() - profilingStart;
    }

    // Loop over all collisions
    if (doProfiling) {
        profilingStart = performance.now();
    }

    // Add all friction eqs
    for (i = 0; i < this.frictionEquations.length; i++) {
        solver.addEquation(this.frictionEquations[i]);
    }

    var ncontacts = contacts.length;
    for (i = 0; i !== ncontacts; i++) {
        solver.addEquation(contacts[i]);
    }

    if (doProfiling) {
        profile.makeContactConstraints = performance.now() - profilingStart;
        profilingStart = performance.now();
    }


    // Add user-added constraints
    var Nconstraints = constraints.length;
    for (i = 0; i !== Nconstraints; i++) {
        var c = constraints[i];
        c.update();
        for (var j = 0, Neq = c.equations.length; j !== Neq; j++) {
            var eq = c.equations[j];
            solver.addEquation(eq);
        }
    }

    // Solve the constrained system
    solver.solve(dt, this);

    if (doProfiling) {
        profile.solve = performance.now() - profilingStart;
    }

    // Remove all contacts from solver
    solver.removeAllEquations();

    // Apply damping, see http://code.google.com/p/bullet/issues/detail?id=74 for details
    var pow = Math.pow;
    N = this.numObjects();
    for (i = 0; i !== N; i++) {
        var bi = bodies[i];
        if (bi.type & DYNAMIC) { // Only for dynamic bodies
            var ld = pow(1.0 - bi.linearDamping, dt);
            var v = bi.velocity;
            v.mult(ld, v);
            var av = bi.angularVelocity;
            if (av) {
                var ad = pow(1.0 - bi.angularDamping, dt);
                av.mult(ad, av);
            }
        }
    }

    // this.dispatchEvent(World_step_preStepEvent);

    // // Invoke pre-step callbacks
    // for(i=0; i!==N; i++){
    //     var bi = bodies[i];
    //     if(bi.preStep){
    //         bi.preStep.call(bi);
    //     }
    // }

    // Leap frog
    // vnew = v + h*f/m
    // xnew = x + h*vnew
    if (doProfiling) {
        profilingStart = performance.now();
    }
    var stepnumber = this.stepnumber;
    var quatNormalize = stepnumber % (this.quatNormalizeSkip + 1) === 0;
    var quatNormalizeFast = this.quatNormalizeFast;

    for (i = 0; i !== N; i++) {
        bodies[i].integrate(dt, quatNormalize, quatNormalizeFast);
    }
    this.clearForces();

    this.broadphase.dirty = true;

    if (doProfiling) {
        profile.integrate = performance.now() - profilingStart;
    }

    // Update world time
    this.time += dt;
    this.stepnumber += 1;

    // this.dispatchEvent(World_step_postStepEvent);

    // // Invoke post-step callbacks
    // for(i=0; i!==N; i++){
    //     var bi = bodies[i];
    //     var postStep = bi.postStep;
    //     if(postStep){
    //         postStep.call(bi);
    //     }
    // }

    // Sleeping update
    if (this.allowSleep) {
        for (i = 0; i !== N; i++) {
            bodies[i].sleepTick(this.time);
        }
    }
};

var additions = [];
var removals = [];
var triggeredEvent = {
    type: 'triggered',
    event: '',
    selfBody: null, // need ?
    otherBody: null, // need ?
    selfShape: null,
    otherShape: null
};
World.prototype.emitTriggeredEvents = function () {

    var id1;
    var id2;

    additions.length = removals.length = 0;
    this.shapeOverlapKeeperExit.getDiff(additions, removals);

    for (var i = 0, l = removals.length; i < l; i += 2) {
        triggeredEvent.event = 'onTriggerExit';
        var shapeA = this.getShapeById(removals[i]);
        var shapeB = this.getShapeById(removals[i + 1]);
        // if(!shapeA.body.isSleeping || !shapeB.body.isSleeping){
        this.triggerMatrix.set(shapeA, shapeB, false);
        triggeredEvent.selfShape = shapeA;
        triggeredEvent.otherShape = shapeB;
        triggeredEvent.selfBody = shapeA.body;
        triggeredEvent.otherBody = shapeB.body;
        shapeA.dispatchEvent(triggeredEvent);

        triggeredEvent.selfShape = shapeB;
        triggeredEvent.otherShape = shapeA;
        triggeredEvent.selfBody = shapeB.body;
        triggeredEvent.otherBody = shapeA.body;
        shapeB.dispatchEvent(triggeredEvent);
        // }
    }

    additions.length = removals.length = 0;
    this.shapeOverlapKeeper.getDiff(additions, removals);
    for (var i = 0, l = additions.length; i < l; i += 2) {
        var id1 = additions[i];
        var id2 = additions[i + 1];
        var shapeA = this.getShapeById(id1);
        var shapeB = this.getShapeById(id2);
        if (this.triggerMatrix.get(shapeA, shapeB)) {
            triggeredEvent.event = 'onTriggerStay';
        } else {
            this.triggerMatrix.set(shapeA, shapeB, true);
            triggeredEvent.event = 'onTriggerEnter';
        }
        triggeredEvent.selfShape = shapeA;
        triggeredEvent.otherShape = shapeB;
        triggeredEvent.selfBody = shapeA.body;
        triggeredEvent.otherBody = shapeB.body;
        shapeA.dispatchEvent(triggeredEvent);

        triggeredEvent.selfShape = shapeB;
        triggeredEvent.otherShape = shapeA;
        triggeredEvent.selfBody = shapeB.body;
        triggeredEvent.otherBody = shapeA.body;
        shapeB.dispatchEvent(triggeredEvent);
    }
};

/**
 * Sets all body forces in the world to zero.
 * @method clearForces
 */
World.prototype.clearForces = function () {
    var bodies = this.bodies;
    var N = bodies.length;
    for (var i = 0; i !== N; i++) {
        var b = bodies[i];
        b.force.set(0, 0, 0);
        b.torque.set(0, 0, 0);
    }
};
