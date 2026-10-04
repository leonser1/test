"""Builds steel_giant/widget_preview.html (artifact page) from desktop/giant.html.
The preview shows a fake desktop with draggable windows that are fed to the giant via window.__pc."""
import pathlib
root = pathlib.Path(__file__).resolve().parent.parent
g = (root / 'desktop' / 'giant.html').read_text(encoding='utf-8')
body = g.split('<body>')[1].split('</body>')[0].strip()
body = body.replace('<canvas id="c"></canvas>', '<canvas id="c" role="img" aria-label="Стальной гигант живёт на рабочем столе: сидит на окнах, играет с мальчиком, сражается с FPV-дронами"></canvas>')
head = '''<title>Стальной гигант · виджет</title>
<style>
/* Layout: fake desktop wallpaper, two draggable app windows, the widget canvas above them, a control strip on top */
:root{--wall-a:#1f4e79;--wall-b:#6aa7d8;--glass:rgba(14,18,26,.62);--edge:rgba(255,255,255,.14);--ink:#eef1f5;--accent:#ffc233;
  --win:#f5f6f8;--win-bar:#e4e7ec;--win-ink:#2b2f36;color-scheme:dark}
html,body{height:100%;margin:0;overflow:hidden;background:linear-gradient(150deg,var(--wall-a),var(--wall-b));color:var(--ink);font:14px system-ui,-apple-system,"Segoe UI",sans-serif}
#c{position:fixed;inset:0;width:100%;height:100%;display:block;pointer-events:none;z-index:5}
.fw{position:fixed;background:var(--win);color:var(--win-ink);border:1px solid rgba(0,0,0,.18);border-radius:8px;box-shadow:0 10px 30px rgba(0,0,0,.35);z-index:1;overflow:hidden;min-width:0}
.fw .bar{height:32px;display:flex;align-items:center;justify-content:space-between;padding:0 6px 0 12px;background:var(--win-bar);cursor:grab;user-select:none;-webkit-user-select:none;touch-action:none;font-size:13px}
.fw .bar:active{cursor:grabbing}
.fw .x{border:0;background:transparent;color:var(--win-ink);font-size:16px;width:30px;height:26px;border-radius:5px;cursor:pointer}
.fw .x:hover{background:#e81123;color:#fff}
.fw .body{padding:12px 14px;font-size:13px;line-height:1.5;color:#4a505a}
.bar-top{position:fixed;top:calc(12px + env(safe-area-inset-top,0px));left:16px;right:16px;display:flex;flex-wrap:wrap;gap:8px;align-items:center;justify-content:space-between;z-index:10;pointer-events:none}
.bar-top>*{pointer-events:auto}
.tag{background:var(--glass);border:1px solid var(--edge);border-radius:9px;padding:8px 12px;backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px);max-width:min(560px,100%)}
.tag b{color:var(--accent)}
.btns{display:flex;flex-wrap:wrap;gap:6px;background:var(--glass);border:1px solid var(--edge);border-radius:9px;padding:5px;backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px)}
.btns button{font:500 13px system-ui,-apple-system,"Segoe UI",sans-serif;color:var(--ink);background:transparent;border:1px solid transparent;border-radius:7px;padding:6px 10px;cursor:pointer}
.btns button:hover{background:rgba(255,255,255,.1)}
button:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
@media (max-width:640px){.tag{display:none}}
</style>
<div class="bar-top">
  <div class="tag">Превью виджета на «рабочем столе». Через 10 секунд начнётся показ нового. Окна можно <b>перетаскивать</b> и <b>закрывать</b>: гигант сидит на них, ездит и падает. Звук включится после клика.</div>
  <div class="btns"><button id="bShow">Всё новое</button><button id="bBoy">Мальчик</button><button id="bFinale">Финал</button><button id="bKnock">Стук в экран</button><button id="bChase">Погоня</button><button id="bBattle">Бой</button><button id="bWin">Новое окно</button><button id="bSummon">Позвать</button></div>
</div>
'''
tail = '''
<script>
(()=>{
  const G=window.__giant,wins=[];let n=0;
  function feed(){window.__pc&&window.__pc.setWindows(wins.filter(w=>w.el.isConnected).map(w=>{const r=w.el.getBoundingClientRect();return{id:w.id,x:r.left,y:r.top,w:r.width,h:r.height}}))}
  function addWin(x,y,w,h,title,text){
    const id='w'+(++n),el=document.createElement('div');el.className='fw';
    el.style.cssText=`left:${x}px;top:${y}px;width:${w}px;height:${h}px`;
    el.innerHTML='<div class="bar"><span></span><button class="x" aria-label="Закрыть окно">×</button></div><div class="body"></div>';
    el.querySelector('.bar span').textContent=title;el.querySelector('.body').textContent=text;
    document.body.appendChild(el);const rec={id,el};wins.push(rec);
    const bar=el.querySelector('.bar');let drag=null;
    bar.addEventListener('pointerdown',e=>{if(e.target.closest('.x'))return;drag={dx:e.clientX-el.offsetLeft,dy:e.clientY-el.offsetTop};bar.setPointerCapture(e.pointerId);
      wins.forEach(o=>o.el.style.zIndex=1);el.style.zIndex=2;wins.splice(wins.indexOf(rec),1);wins.unshift(rec);feed()});
    bar.addEventListener('pointermove',e=>{if(!drag)return;el.style.left=Math.max(-w+60,Math.min(innerWidth-60,e.clientX-drag.dx))+'px';el.style.top=Math.max(0,Math.min(innerHeight-40,e.clientY-drag.dy))+'px';feed()});
    const end=()=>{drag=null};bar.addEventListener('pointerup',end);bar.addEventListener('pointercancel',end);
    el.querySelector('.x').addEventListener('click',()=>{el.remove();wins.splice(wins.indexOf(rec),1);feed()});
    feed();
  }
  const W=innerWidth,H=innerHeight,s=Math.min(1,W/1300);
  addWin(Math.round(W*0.08),Math.round(H*0.42),Math.round(480*s),Math.round(300*s),'Заметки','Список дел: купить батарейки, починить радио, не говорить никому про гиганта.');
  addWin(Math.round(W*0.56),Math.round(H*0.26),Math.round(440*s),Math.round(360*s),'Проводник','Документы · Загрузки · Рабочий стол');
  addEventListener('resize',feed);
  const on=(id,f)=>document.getElementById(id).addEventListener('click',f);
  on('bShow',()=>G.showcase());on('bBoy',()=>G.boy());
  setTimeout(()=>G.showcase(),9000);on('bFinale',()=>G.finale());on('bKnock',()=>G.knock());on('bChase',()=>G.chase());on('bBattle',()=>G.battle());on('bSummon',()=>G.summon());
  on('bWin',()=>addWin(Math.round(Math.random()*(W-420)),Math.round(H*0.2+Math.random()*H*0.4),Math.round(380*s),Math.round(240*s),'Окно','Новое окно для гиганта.'));
})();
</script>
'''
(root / 'widget_preview.html').write_text(head + body + tail, encoding='utf-8')
print('ok')
