"""Export lossless JPEG-frame playback from an episode to a standalone HTML file."""

import argparse
import base64
import json
from pathlib import Path

import h5py

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("episode")
parser.add_argument("--output", required=True)
args = parser.parse_args()
with h5py.File(args.episode) as f:
    if not f.attrs.get("complete", False):
        raise ValueError("Incomplete episode; inspect end_reason before replay")
    meta = json.loads(f.attrs["metadata_json"])
    payload = dict(
        instruction=f.attrs["instruction"],
        robot=f.attrs["robot"],
        cameras=meta["cameras"],
        times=f["frames/sim_time"][:].tolist(),
        images=[
            [base64.b64encode(raw.tobytes()).decode("ascii") for raw in f[f"images/camera_{i}"]]
            for i in range(len(meta["cameras"]))
        ],
    )
# Embedded JPEG bytes are unchanged. Escape '<' so metadata cannot terminate the script element.
data = json.dumps(payload, ensure_ascii=False).replace("<", "\\u003c")
html = """<!doctype html><meta charset="utf-8"><title>HDF5 episode replay</title>
<style>body{font-family:sans-serif;margin:24px;background:#fafafa}#views{display:flex;flex-wrap:wrap;gap:16px}img{max-width:480px;width:100%}input{width:70%}</style>
<h2 id="title"></h2><p id="instruction"></p><button id="play">播放 / 暂停</button>
<input id="frame" type="range" min="0" value="0"><span id="time"></span><div id="views"></div>
<script id="data" type="application/json">PAYLOAD</script>
<script>
const d=JSON.parse(document.getElementById('data').textContent), slider=document.getElementById('frame');

document.getElementById('title').textContent=d.robot;

document.getElementById('instruction').textContent=d.instruction;

slider.max=Math.max(0,d.times.length-1);

const imgs=d.cameras.map(name=>{const box=document.createElement('div'),
title=document.createElement('h3'),img=document.createElement('img');
title.textContent=name;
box.append(title,img);
document.getElementById('views').append(box);
return img;
});

let playing=false,timer;

function show(){const i=Number(slider.value);
imgs.forEach((img,c)=>img.src='data:image/jpeg;base64,'+d.images[c][i]);
document.getElementById('time').textContent=`帧 ${i} / ${d.times.length} · ${d.times[i]?.toFixed(3)} 仿真秒`;
}
function next(){if(!playing)return;
const i=Number(slider.value);
if(i>=d.times.length-1){playing=false;
return;
}timer=setTimeout(()=>{slider.value=i+1;
show();
next();
},Math.max(0,(d.times[i+1]-d.times[i])*1000));
}
document.getElementById('play').onclick=()=>{playing=!playing;
clearTimeout(timer);
if(playing){if(Number(slider.value)>=d.times.length-1)slider.value=0;
show();
next();
}};

slider.oninput=()=>{playing=false;
clearTimeout(timer);
show();
};
if(d.times.length)show();

</script>""".replace("PAYLOAD", data)
Path(args.output).write_text(html)
print(args.output)
