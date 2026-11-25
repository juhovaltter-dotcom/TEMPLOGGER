// Offline shell: after one visit with internet, the app opens without a connection.
// Network first, so an online visit always refreshes the cached copy. Change VERSION only to drop old caches.
const VERSION = 'cottage-logger-v1';
const SHELL = ['./', 'index.html', 'manifest.webmanifest', 'icon-192.png', 'icon-512.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(VERSION).then((c) => c.addAll(SHELL)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys().then((ks) => Promise.all(ks.filter((k) => k !== VERSION).map((k) => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  if (e.request.method !== 'GET') return;
  // Network first (fresh copy when online), but never let a bad or slow network break the app:
  // an error page (e.g. 404 if Pages is switched off), no connection, or a connection that
  // hangs for over 4 s all fall back to the saved copy.
  const cached = () => caches.match(e.request).then((m) => m || caches.match('index.html'));
  const net = fetch(e.request).then((r) => {
    if (r.ok && new URL(e.request.url).origin === location.origin) { const copy = r.clone(); caches.open(VERSION).then((c) => c.put(e.request, copy)); }
    return r;
  });
  const slow = new Promise((_, no) => setTimeout(() => no(new Error('slow')), 4000));
  e.respondWith(
    Promise.race([net, slow]).then((r) => (r.ok ? r : cached().then((m) => m || r))).catch(() => cached().then((m) => m || net))
  );
});
