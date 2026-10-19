import fs from 'node:fs';
import path from 'node:path';
import { getDocument } from './node_modules/pdfjs-dist/legacy/build/pdf.mjs';

const [pdfPath, startArg, endArg, outputDir] = process.argv.slice(2);
if (!pdfPath || !startArg || !endArg || !outputDir) {
  throw new Error('Usage: node extract-pages.mjs PDF startPage endPage outputDir');
}

fs.mkdirSync(outputDir, { recursive: true });
const data = new Uint8Array(fs.readFileSync(pdfPath));
const pdf = await getDocument({ data, disableWorker: true }).promise;

for (let pageNo = Number(startArg); pageNo <= Number(endArg); pageNo += 1) {
  const page = await pdf.getPage(pageNo);
  const content = await page.getTextContent();
  let line = '';
  let lastY = null;
  const lines = [];
  for (const item of content.items) {
    if (!('str' in item)) continue;
    const y = item.transform?.[5] ?? null;
    if (lastY !== null && y !== null && Math.abs(y - lastY) > 2) {
      if (line.trim()) lines.push(line.trim());
      line = '';
    }
    line += `${item.str} `;
    lastY = y;
  }
  if (line.trim()) lines.push(line.trim());
  const target = path.join(outputDir, `page-${String(pageNo).padStart(3, '0')}.txt`);
  fs.writeFileSync(target, `${lines.join('\n')}\n`, 'utf8');
}

console.log(`Extracted ${Number(endArg) - Number(startArg) + 1} pages to ${outputDir}`);
