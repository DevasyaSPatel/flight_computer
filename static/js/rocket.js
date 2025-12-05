// Three.js Setup
const container = document.getElementById('rocket-container');
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });

renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 5, 5);
scene.add(directionalLight);

// Rocket Group
const rocketGroup = new THREE.Group();
scene.add(rocketGroup);

// Load Custom Model or Fallback to Procedural
const objLoader = new THREE.OBJLoader();
const gltfLoader = new THREE.GLTFLoader();

// Try loading a custom model (assuming user puts one in static/assets/rocket.obj)
// Since we can't check file existence easily from client side without 404, we try and catch.
// For this demo, we will default to a procedural rocket if loading fails (which it will initially).

function createProceduralRocket() {
    // Body
    const bodyGeo = new THREE.CylinderGeometry(0.5, 0.5, 4, 32);
    const bodyMat = new THREE.MeshPhongMaterial({ color: 0xdddddd });
    const body = new THREE.Mesh(bodyGeo, bodyMat);

    // Nose Cone
    const noseGeo = new THREE.ConeGeometry(0.5, 1, 32);
    const noseMat = new THREE.MeshPhongMaterial({ color: 0xff0000 });
    const nose = new THREE.Mesh(noseGeo, noseMat);
    nose.position.y = 2.5;

    // Fins
    const finGeo = new THREE.BoxGeometry(0.1, 1, 1);
    const finMat = new THREE.MeshPhongMaterial({ color: 0x333333 });

    const fin1 = new THREE.Mesh(finGeo, finMat);
    fin1.position.set(0.5, -1.5, 0);

    const fin2 = new THREE.Mesh(finGeo, finMat);
    fin2.position.set(-0.5, -1.5, 0);

    const fin3 = new THREE.Mesh(finGeo, finMat);
    fin3.position.set(0, -1.5, 0.5);
    fin3.rotation.y = Math.PI / 2;

    const fin4 = new THREE.Mesh(finGeo, finMat);
    fin4.position.set(0, -1.5, -0.5);
    fin4.rotation.y = Math.PI / 2;

    rocketGroup.add(body);
    rocketGroup.add(nose);
    rocketGroup.add(fin1);
    rocketGroup.add(fin2);
    rocketGroup.add(fin3);
    rocketGroup.add(fin4);
}

// Attempt to load 'rocket.obj' from assets
objLoader.load(
    '/static/assets/rocket.obj',
    (object) => {
        // Success
        object.scale.set(0.5, 0.5, 0.5); // Adjust scale as needed
        rocketGroup.add(object);
    },
    (xhr) => {
        // Progress
    },
    (error) => {
        // Error or file not found -> Use procedural
        console.log('Custom model not found, using procedural rocket.');
        createProceduralRocket();
    }
);

camera.position.z = 7;
camera.position.y = 1;

// Animation Loop
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();

// Handle Resize
window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
});

// Update Orientation from Telemetry
window.updateRocketOrientation = function (orientation) {
    // Smooth interpolation could be added here
    rocketGroup.rotation.x = orientation.pitch;
    rocketGroup.rotation.y = orientation.yaw; // Yaw usually around Y axis in 3D
    rocketGroup.rotation.z = orientation.roll;
};
