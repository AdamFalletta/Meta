# AFI Mission Control OS

A modular rebuild of the single-file `afi-mc-beta1c.html` dashboard. Same dark mission-control look, but split into ES modules with a plugin/service API.

## Run it

ES modules do not load from `file://`, so use the tiny included server:

```powershell
cd C:\Users\Owner\Desktop\AFI
node serve.js
```

Then open <http://localhost:8080>.

On Windows you can also double-click `start.bat`. There are no npm dependencies.

## What changed from the single-file version

- **State sync loop fixed.** The old `BroadcastChannel` handler persisted incoming state and re-broadcast it, so two open tabs could ping-pong writes forever. Incoming state is now applied and persisted without being re-broadcast.
- **Scheduler actually runs.** Enabled tasks fire on their interval. `Run` executes one immediately. Status, last-run time and run count are tracked. Only one elected tab runs timers, so multiple open windows do not double-fire webhooks.
- **Pop-outs are real pages.** `wall.html` and `monitor.html` load the same modules and subscribe to state changes, so they stay live. All render engines work in pop-outs (not just Template HTML).
- **WebSockets implemented.** `EndpointService.openSocket()` opens, parses JSON frames, and closes sockets cleanly.
- **Modular structure.** State, persistence, sync, services, renderers, UI and plugins are separate files.
- **Safer rendering.** User values are HTML-escaped before being placed in tables and form fields. Custom module code is still arbitrary code by design (see security note).

## Project structure

```text
AFI/
  index.html               main wall
  wall.html                live pop-out wall
  monitor.html             live single-monitor pop-out
  serve.js                 zero-dependency static server
  styles/
    base.css               reset, theme, header
    wall.css               wall grid and monitor chrome
    components.css         drawer, forms, tables
    modals.css             modal shell
    extras.css             status pills, toasts, pop-outs, plugin UI
  src/
    main.js                boot the main window
    app.js                 App class (state + registries + event bus)
    bootstrap.js           wire built-in services, renderers and plugins
    actions.js             all user-facing state mutations (CRUD)
    core/
      constants.js         storage names, event names, grid presets
      event-bus.js         tiny pub/sub
      persistence.js       IndexedDB adapter
      state.js             state manager, revisions, remote apply
      state-sync.js        BroadcastChannel wrapper (no re-broadcast)
      selectors.js         active layout / monitor helpers
      leader.js            scheduler leader election
      service-registry.js
      renderer-registry.js
      plugin-registry.js
      dom.js               escaping, ids, downloads, grid sizing
    data/
      default-state.js     demo data + schema normalization
    services/
      endpoint-service.js  REST fetch/cache + WebSockets
      scheduler-service.js interval runner + webhook POST/GET
      popout-service.js    opens wall.html / monitor.html
    renderers/
      index.js             register built-ins
      template-html.js
      canvas-js.js
      inline.js
      static-text.js
      raw-html.js
      iframe.js
    ui/
      index.js             install all UI modules
      header.js            project/layout selectors, clock, pop-out menu
      wall.js              main wall events
      wall-view.js         shared wall renderer (main + pop-outs)
      drawer.js            management console tabs and CRUD forms
      modals.js            monitor config + entity editor
      toast.js             toast notifications
    plugins/
      system-info.js       example plugin that adds an Extensions tab
    popout/
      wall.js              pop-out wall entry point
      monitor.js           pop-out monitor entry point
```

## Adding a service

Register an object with `start(app)` and optional `stop()` in `src/bootstrap.js`:

```js
import { MyService } from './services/my-service.js';

app.registerService('my-service', new MyService());
```

```js
export class MyService {
  start(app) {
    this.app = app;
    this.timer = setInterval(() => {
      console.log('service tick', app.data.activeProjectId);
    }, 5000);
  }

  stop() {
    clearInterval(this.timer);
  }
}
```

Read it anywhere with `app.services.get('my-service')`.

## Adding a render engine

A renderer is any object with an `id`, a `label` and a `render(context)` function. It automatically appears in the Module editor.

```js
app.registerRenderer('My Renderer', {
  label: 'My Renderer',
  render({ container, module, runtime, state, app }) {
    container.textContent = `Hello from ${module.name}`;
    // optional cleanup
    return () => console.log('removed');
  },
});
```

`runtime` gives you `endpoints`, `fetchEndpointData(id)` and `fetchEndpoint(id, options)`.

## Adding a plugin

Put a file in `src/plugins/` and import it in `src/bootstrap.js`:

```js
import { myPlugin } from './plugins/my-plugin.js';
app.use(myPlugin);
```

A plugin can register drawer tabs, renderers and services, and subscribe to events:

```js
export const myPlugin = {
  id: 'my-plugin',
  name: 'My Plugin',
  setup(app) {
    app.registerDrawerTab({
      id: 'my-tab',
      label: 'My Tab',
      order: 95,
      render(container, { app, refresh }) {
        container.innerHTML = `<button data-action="do-thing">Do thing</button>`;
      },
    });

    app.bus.on('state:change', ({ reason }) => {
      console.log('state changed because', reason);
    });
  },
};
```

The example `system-info` plugin adds an **Extensions** tab that lists loaded plugins, services and renderers.

## Data and backups

State is stored in IndexedDB:

- database: `AFI_MISSION_CONTROL_DB`
- store: `state_store`
- keys: `current_state`, `current_meta`

The schema normalizer accepts older backups and fills in missing fields. Use **System/IO → Export JSON Backup** and **Import JSON Backup** to move data.

If you had data in the old `file://` version, open the old file, export JSON, then import it here. Browsers keep `file://` and `http://localhost` storage separate.

## Notes

- **Security:** module code is executed with `new Function`. That is the point of the product, but it means you should not import backups or modules from people you do not trust.
- **Scheduler defaults:** the demo `Telemetry Sync` task is disabled and points at a placeholder URL. Edit it to a real webhook and enable it, or use **Run** for a one-off test.
- **CORS:** browser REST calls still need the endpoint to allow cross-origin requests. The endpoint test button reports failures via toast.
