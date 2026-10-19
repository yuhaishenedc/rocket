import fs from 'node:fs';
import path from 'node:path';
import { createCanvas } from '@napi-rs/canvas';
import { getDocument } from './node_modules/pdfjs-dist/legacy/build/pdf.mjs';

const [pdfPath, startArg, endArg, outputDir, scaleArg = '2'] = process.argv.slice(2);
if (!pdfPath || !startArg || !endArg || !outputDir) {
  throw new Error('Usage: node render-pages.mjs PDF startPage endPage outputDir [scale]');
}

fs.mkdirSync(outputDir, { recursive: true });
const data = new Uint8Array(fs.readFileSync(pdfPath));
const pdf = await getDocument({ data, disableWorker: true }).promise;
const scale = Number(scaleArg);

for (let pageNo = Number(startArg); pageNo <= Number(endArg); pageNo += 1) {
  const page = await pdf.getPage(pageNo);
  const viewport = page.getViewport({ scale });
  const canvas = createCanvas(Math.ceil(viewport.width), Math.ceil(viewport.height));
  const context = canvas.getContext('2d');
  await page.render({ canvasContext: context, viewport }).promise;
  const target = path.join(outputDir, `page-${String(pageNo).padStart(3, '0')}.png`);
  fs.writeFileSync(target, canvas.toBuffer('image/png'));
}

console.log(`Rendered ${Number(endArg) - Number(startArg) + 1} pages to ${outputDir}`);
