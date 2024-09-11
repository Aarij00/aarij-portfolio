

var app = document.getElementById('type-text');

var typewriter = new Typewriter(app, {
    loop: true,
    delay:50,
    deleteSpeed:5
});

typewriter
    .typeString('nice to meet you.')
    .pauseFor(1000)
    .deleteAll()
    .pauseFor(500)
    .typeString("there are 10 types of people in this world...")
    .pauseFor(1000)
    .deleteAll()
    .pauseFor(500)
    .typeString("those who understand binary and those who don't")
    .pauseFor(1000)
    .deleteAll()
    .pauseFor(500)
    .typeString("send me a message and lets connect!")
    .pauseFor(1000)
    .deleteAll()
    .pauseFor(500)
    .typeString("thanks for visiting!")
    .pauseFor(1000)
    .deleteAll()
    .start();

