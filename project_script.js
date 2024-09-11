// Function to get URL parameters
function getQueryParam(name) {
    const params = new URLSearchParams(window.location.search);
    return params.get(name);
}

// Function to display project details based on the query parameter
function displayProjectInfo() {
    const projectId = getQueryParam('project');

    // Example data for each project /Users/aarij/Documents/aarij-portfolio/project_script.js
    const projectData = {
        1: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
        },
        2: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
        },
        3: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
        },
        4: {
            title: "/aarij/projects/<span class='highlight'>[project title]</span>",
            highlight: "[project title]",
            description: "Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!Lorem ipsum dolor sit amet consectetur adipisicing elit. Aperiam animi dolor fuga tempore maiores voluptates laborum quos earum expedita delectus esse, consectetur laboriosam numquam quae accusantium cum culpa repudiandae voluptate!",
            conceptImages: ["test.jpeg", "test.jpeg", "test.jpeg"],
            video: "interaction-video1.mp4",
            arduinoCode: `// Arduino code for Project 1\nvoid setup() {}\nvoid loop() {}`,
            circuitSchematic: "circuit.png",
            references: ["reference1.pdf", "reference2.pdf"]
        },
    };

    const project = projectData[projectId];

    if (project) {
        document.getElementById('project-title').innerHTML = `${project.title}`;
        document.getElementById('project-desc').innerHTML = project.description;

        const conceptImagesContainer = document.getElementById('concept-images-container');
        conceptImagesContainer.innerHTML = project.conceptImages.map(src => 
            `<img src="${src}" alt="Concept Image" class="concept-image">`
        ).join('');

        document.getElementById('project-video').src = project.video;

        document.getElementById('arduino-code').textContent = project.arduinoCode;

        document.getElementById('circuit-schematic').src = project.circuitSchematic;

        const referenceMaterials = document.getElementById('reference-materials');
        referenceMaterials.innerHTML = project.references.map(ref => 
            `<li><a href="${ref}" target="_blank">${ref}</a></li>`
        ).join('');
    } else {
        document.getElementById('project-info').innerHTML = `
            <h1>Project Not Found</h1>
        `;
    }
}

// Execute the function on page load
document.addEventListener('DOMContentLoaded', displayProjectInfo);
