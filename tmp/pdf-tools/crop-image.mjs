import fs from 'node:fs';
import { createCanvas, loadImage } from '@napi-rs/canvas';

const [source, target, xArg, yArg, widthArg, heightArg] = process.argv.slice(2);
if (!heightArg) throw new Error('Usage: node crop-image.mjs source target x y width height');
const [x, y, width, height] = [xArg, yArg, widthArg, heightArg].map(Number);
const image = await loadImage(source);
const canvas = createCanvas(width, height);
canvas.getContext('2d').drawImage(image, x, y, width, height, 0, 0, width, height);
fs.writeFileSync(target, canvas.toBuffer('image/png'));
