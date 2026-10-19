import fs from 'node:fs';
import { getDocument } from './node_modules/pdfjs-dist/legacy/build/pdf.mjs';

const [pdfPath, startArg, endArg] = process.argv.slice(2);
if (!pdfPath) throw new Error('Usage: node inspect-pdf.mjs PDF [startPage] [endPage]');

const data = new Uint8Array(fs.readFileSync(pdfPath));
const pdf = await getDocument({ data, disableWorker: true }).promise;
const start = startArg ? Number(startArg) : 1;
const end = endArg ? Number(endArg) : Math.min(pdf.numPages, start);

console.log(`PAGES ${pdf.numPages}`);
const outline = await pdf.getOutline();
if (outline) {
  console.log('OUTLINE');
  const walk = (items, depth = 0) => {
    for (const item of items) {
      console.log(`${'  '.repeat(depth)}${item.title}`);
      if (item.items?.length) walk(item.items, depth + 1);
    }
  };
  walk(outline);
}

for (let pageNo = start; pageNo <= end; pageNo += 1) {
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
  console.log(`\n===== PDF_PAGE_${pageNo} =====`);
  console.log(lines.join('\n'));
}
