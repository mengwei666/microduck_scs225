#!/usr/bin/env python3
"""item_placer.py — 在 MuJoCo 窗口里挑选/摆放物品并保存新位置。

用法:
    uv run python scripts/item_placer.py \
        --xml src/mjlab_microduck/robot/microduck/items_showcase.xml \
        --out /tmp/placed_scene.xml

按键:
    1..9/0     选择物品(按物品序号;窗口标题显示当前选中)
    ↑ ↓        沿世界 +x / -x 平移
    ← →        沿世界 +y / -y 平移
    PageUp/Down 沿 +z / -z 抬升/降低
    R          复位选中物品到初始位置
    S          保存:写 placed_items.json 并生成 --out 新 XML
    Q / Esc    退出(不保存)

说明:物品都是 freejoint,选中后直接改写其 qpos 平移;保存时把新
位置写回 XML 的 <body pos=...>。视角用鼠标随意转动,平移一律按
世界坐标轴(x 朝前,y 朝左,z 向上)。
"""
import argparse
import json
import re
import sys

import glfw
import mujoco


def body_name(model, b):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)


def find_item_bodies(model):
    """世界直接子 body 且带 freejoint 的 -> (名字, 对应 qpos 地址)。"""
    items = []
    for b in range(model.nbody):
        if model.body_parentid[b] != 0:
            continue
        jnt = model.body_jntadr[b]
        if jnt >= 0 and model.jnt_type[jnt] == mujoco.mjtJoint.mjJNT_FREE:
            items.append((body_name(model, b), model.jnt_qposadr[jnt]))
    return items


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xml", required=True, help="源场景 XML")
    ap.add_argument("--out", default="/tmp/placed_scene.xml", help="保存的新 XML")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    items = find_item_bodies(model)
    if not items:
        sys.exit("没有找到带 freejoint 的物品 (world 直接子 body)")
    names = [n for n, _ in items]
    addr = dict(items)  # name -> qpos address
    initial = {n: data.qpos[a:a + 3].copy() for n, a in items}

    def qpos_of(name):
        a = addr[name]
        return data.qpos[a:a + 3]

    if not glfw.init():
        sys.exit("glfw init failed")
    window = glfw.create_window(1200, 800, "item placer", None, None)
    glfw.make_context_current(window)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance = 6.0
    cam.elevation = -30
    cam.azimuth = 45
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)

    sel = 0

    def set_sel(i):
        nonlocal sel
        name = names[i]
        sel = i
        hi = dict(initial)[name].copy()
        glfw.set_window_title(window, f"item placer — [{i}] {name}  pos=({hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f})  [S]保存 [Q]退出")

    def highlight():
        sel_name = names[sel]
        rgb = model.geom_rgba.copy()
        for b in range(model.nbody):
            g0, n = model.body_geomadr[b], model.body_geomnum[b]
            if body_name(model, b) == sel_name:
                for g in range(g0, g0 + n):
                    rgb[g, 0], rgb[g, 1], rgb[g, 2] = 1.0, 0.9, 0.1
                    rgb[g, 3] = 1.0
        return rgb

    def key_cb(window, key, scancode, action, mods):
        nonlocal sel
        if action != glfw.PRESS:
            return
        print(f"[key] key={key} mods={mods} sel={sel} name={names[sel]}", flush=True)
        name = names[sel]
        step = 0.05
        if glfw.keys[glfw.KEY_LEFT_SHIFT] if False else (mods & glfw.MOD_SHIFT):
            step = 0.005  # 按住 Shift 微调
        pos = qpos_of(name)
        if glfw.KEY_UP == key:
            pos[0] += step
        elif glfw.KEY_DOWN == key:
            pos[0] -= step
        elif glfw.KEY_LEFT == key:
            pos[1] += step
        elif glfw.KEY_RIGHT == key:
            pos[1] -= step
        elif glfw.KEY_PAGE_UP == key:
            pos[2] += step
        elif glfw.KEY_PAGE_DOWN == key:
            pos[2] -= step
        elif glfw.KEY_R == key:
            data.qpos[addr[name]:addr[name] + 3] = initial[name]
        elif glfw.KEY_S == key:
            save()
            return
        elif glfw.KEY_Q == key or glfw.KEY_ESCAPE == key:
            glfw.set_window_should_close(window, True)
            return
        elif glfw.KEY_0 <= key <= glfw.KEY_9:
            i = int(key - glfw.KEY_0)
            if i < len(names):
                set_sel(i)
            return
        glfw.set_window_title(window,
            f"item placer — [{sel}] {name}  pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})  [S]保存 [Q]退出")

    def save():
        out_json = args.out + ".json"
        poses = {n: qpos_of(n).tolist() for n, _ in items}
        with open(out_json, "w") as f:
            json.dump(poses, f, indent=2)
        text = open(args.xml).read()
        for n, _ in items:
            p = poses[n]
            text = re.sub(
                rf'(<body name="{n}"[^>]*\bpos=")[^"]*(")', rf'\g<1>{p[0]} {p[1]} {p[2]}\g<2>', text, count=1)
        with open(args.out, "w") as f:
            f.write(text)
        print(f"已保存: {out_json} + {args.out}")

    glfw.set_key_callback(window, key_cb)
    scene = mujoco.MjvScene(model, 1000)
    ctx = mujoco.MjrContext(model, 100)
    model.geom_rgba = highlight()
    mujoco.mj_forward(model, data)

    while not glfw.window_should_close(window):
        rgba = highlight()
        model.geom_rgba = rgba
        mujoco.mj_forward(model, data)  # 同步 qpos 修改到渲染位置(摆放时不做物理步进)
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        viewport = mujoco.MjrRect(0, 0, 1200, 800)
        mujoco.mjr_render(viewport, scene, ctx)
        glfw.swap_buffers(window)
        glfw.poll_events()
        if glfw.window_should_close(window):
            break

    glfw.terminate()
    print("提示: 未保存的改动已丢弃 (按 S 才会写盘)")


if __name__ == "__main__":
    main()