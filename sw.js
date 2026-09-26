const CACHE='simulink-v1.7.1';
const CHART_JS='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.5.1/chart.umd.min.js';
const OFFLINE=['./index.html','./manifest.webmanifest','./icon.svg','./styles.css?v=1.7.1','./app.js?v=1.7.1'];

self.addEventListener('install',event=>{
  event.waitUntil(
    caches.open(CACHE).then(async cache=>{
      await cache.addAll(OFFLINE);
      try{
        const response=await fetch(CHART_JS,{mode:'cors'});
        await cache.put(CHART_JS,response);
      }catch(e){}
    }).then(()=>self.skipWaiting())
  );
});

self.addEventListener('activate',event=>{
  event.waitUntil(
    caches.keys()
      .then(keys=>Promise.all(keys.filter(key=>key!==CACHE).map(key=>caches.delete(key))))
      .then(()=>self.clients.claim())
  );
});

self.addEventListener('fetch',event=>{
  if(event.request.method!=='GET')return;
  const url=new URL(event.request.url);

  if(event.request.url===CHART_JS){
    event.respondWith(
      caches.match(CHART_JS).then(cached=>{
        if(cached)return cached;
        return fetch(event.request).then(response=>{
          const copy=response.clone();
          caches.open(CACHE).then(cache=>cache.put(CHART_JS,copy));
          return response;
        });
      })
    );
    return;
  }

  if(url.origin!==location.origin)return;

  event.respondWith(
    fetch(event.request,{cache:'no-store'}).then(response=>{
      const copy=response.clone();
      caches.open(CACHE).then(cache=>cache.put(event.request,copy));
      return response;
    }).catch(()=>caches.match(event.request).then(r=>r||caches.match('./index.html')))
  );
});