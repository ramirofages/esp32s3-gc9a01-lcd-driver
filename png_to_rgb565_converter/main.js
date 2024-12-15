const fs = require('fs');
const { PNG } = require('pngjs');

function convertToRGB565(r, g, b) {
    // Convert 8-bit RGB to 16-bit RGB565
    const r5 = (r >> 3) & 0x1F; // 5 bits for red
    const g6 = (g >> 2) & 0x3F; // 6 bits for green
    const b5 = (b >> 3) & 0x1F; // 5 bits for blue
    return (r5 << 11) | (g6 << 5) | b5;
}

function processImage(inputPath, outputPath) {
    fs.createReadStream(inputPath)
        .pipe(new PNG())
        .on('parsed', function () {
            const rgb565Array = [];
            for (let y = this.height - 1; y >= 0; y--) { // Start from the last row and go up
                for (let x = 0; x < this.width; x++) {
                    const idx = (this.width * y + x) << 2; // Calculate RGBA index
                    const r = this.data[idx];
                    const g = this.data[idx + 1];
                    const b = this.data[idx + 2];
                    const rgb565 = convertToRGB565(r, g, b);
                    rgb565Array.push(rgb565);
                }
            }

            // Write the RGB565 data to a binary file
            const buffer = Buffer.alloc(rgb565Array.length * 2); // Each RGB565 is 2 bytes
            for (let i = 0; i < rgb565Array.length; i++) {
                buffer.writeUInt16LE(rgb565Array[i], i * 2);
            }
            fs.writeFileSync(outputPath, buffer);
            console.log(`RGB565 data written to ${outputPath}`);
        })
        .on('error', (err) => {
            console.error(`Error processing the image: ${err.message}`);
        });
}

// Get command-line arguments
const args = process.argv.slice(2);
if (args.length < 2) {
    console.error('Usage: node script.js <inputFile> <outputFile>');
    process.exit(1);
}

const [inputPng, outputFile] = args;
processImage(inputPng, outputFile);