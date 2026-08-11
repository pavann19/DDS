const puppeteer = require('puppeteer');
(async () => {
  const browser = await puppeteer.launch();
  const page = await browser.newPage();
  await page.setViewport({ width: 1280, height: 800 });
  await page.goto('http://localhost:3000');
  await page.waitForTimeout(10000); // wait for 10s for app to load and ws to connect
  await page.screenshot({ path: 'screenshot.png' });
  await browser.close();
})();
