// LocateBot service worker: caches only the app shell. Live data (/state, /stream, /inbox, /manual...) always
// goes to the network, so nothing stale ever drives the car.
const CACHE = "locatebot-v2";
const SHELL = ["/", "/manifest.webmanifest", "/icon-192.png", "/icon-512.png", "/fonts/PixelifySans.woff2"];
self.addEventListener("install", e => { e.waitUntil(caches.open(CACHE).then(c => c.addAll(SHELL))); self.skipWaiting(); });
self.addEventListener("activate", e => { e.waitUntil(self.clients.claim()); });
self.addEventListener("fetch", e => {
  const url = new URL(e.request.url);
  if (e.request.method !== "GET" || !SHELL.includes(url.pathname)) return;     // live endpoints: network only
  e.respondWith(fetch(e.request).catch(() => caches.match(e.request)));
});
