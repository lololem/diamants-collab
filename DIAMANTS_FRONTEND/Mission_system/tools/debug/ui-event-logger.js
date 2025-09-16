/*
 UI Event Logger for DIAMANTS V3
 - Delegated listeners on the control panel (#ros_interface)
 - Logs button clicks, select changes, inputs (range/number/text)
 - Outputs structured logs consumable by Console Ninja
*/

(function () {
    const NS = 'UI';
    const PREFIX = '🥷 UI';
    const start = Date.now();

    function nowMs() {
        return Date.now() - start;
    }

    function getSectionName(el) {
        try {
            const panel = el.closest('.ros_panel');
            const h3 = panel ? panel.querySelector('h3') : null;
            return (h3 && h3.textContent.trim()) || 'Panel';
        } catch (_) {
            return 'Panel';
        }
    }

    function asText(el) {
        if (!el) return '';
        const t = (el.textContent || '').trim().replace(/\s+/g, ' ');
        return t.length > 80 ? t.slice(0, 77) + '…' : t;
    }

    function log(event, data) {
        const payload = { t: nowMs(), event, ...data };
        // Route to global debug bus if present
        try { window.DIAMANT_DEBUG && window.DIAMANT_DEBUG.logEvent && window.DIAMANT_DEBUG.logEvent(NS, event, data); } catch(_) {}
        // Plain console for Console Ninja
        try { console.log(PREFIX, payload); } catch(_) {}
    }

    function handleClick(e) {
        const t = e.target.closest('button, a, .btn_ros, .ros_button');
        if (!t) return;
        const section = getSectionName(t);
        log('click', {
            section,
            id: t.id || null,
            class: t.className || null,
            text: asText(t)
        });
    }

    function handleChange(e) {
        const t = e.target;
        if (!(t instanceof HTMLElement)) return;
        if (!['SELECT', 'INPUT', 'TEXTAREA'].includes(t.tagName)) return;
        const section = getSectionName(t);
        log('change', {
            section,
            id: t.id || null,
            type: t.getAttribute('type') || t.tagName.toLowerCase(),
            value: (t.type === 'password') ? '***' : t.value
        });
    }

    let inputThrottleId = null;
    function handleInput(e) {
        const t = e.target;
        if (!(t instanceof HTMLElement)) return;
        if (t.tagName !== 'INPUT') return;
        const type = (t.getAttribute('type') || '').toLowerCase();
        if (!['range', 'number', 'text'].includes(type)) return;
        if (inputThrottleId) return; // simple throttle to avoid floods
        inputThrottleId = setTimeout(() => { inputThrottleId = null; }, 100);
        const section = getSectionName(t);
        log('input', {
            section,
            id: t.id || null,
            type,
            value: t.value
        });
    }

    function bindGlobalExtras() {
        const toggleBtn = document.getElementById('toggle_panel');
        if (toggleBtn && !toggleBtn.__uiLogged) {
            toggleBtn.addEventListener('click', () => {
                log('click', { section: 'Layout', id: 'toggle_panel', text: 'Toggle Panel' });
            }, true);
            toggleBtn.__uiLogged = true;
        }

        const minimapBtn = Array.from(document.querySelectorAll('button.ros_button'))
            .find(b => (b.textContent || '').includes('Minimap'));
        if (minimapBtn && !minimapBtn.__uiLogged) {
            minimapBtn.addEventListener('click', () => {
                log('click', { section: 'System', id: minimapBtn.id || null, text: 'Minimap' });
            }, true);
            minimapBtn.__uiLogged = true;
        }
    }

    function init() {
        const panel = document.getElementById('ros_interface');
        if (!panel) {
            console.warn('UIEventLogger: #ros_interface introuvable');
            return;
        }
        if (panel.__uiLoggerBound) return;
        panel.addEventListener('click', handleClick, true);
        panel.addEventListener('change', handleChange, true);
        panel.addEventListener('input', handleInput, true);
        panel.__uiLoggerBound = true;
        bindGlobalExtras();
        log('logger-ready', { section: 'bootstrap' });
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init, { once: true });
    } else {
        init();
    }
})();
