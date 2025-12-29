async function main() {
  const statusEl = document.getElementById('status');
  const showExternalEl = document.getElementById('show-external');
  const showNoFindingsEl = document.getElementById('show-no-findings');
  const sidebarEl = document.getElementById('sidebar');

  function setStatus(msg) {
    if (statusEl) statusEl.textContent = msg;
  }

  setStatus('Loading graph…');

  let graph = null;
  const embedded = document.getElementById('graph-data');
  if (embedded && embedded.textContent && embedded.textContent.trim().length > 0) {
    graph = JSON.parse(embedded.textContent);
  } else {
    // Fallback for server-based viewing.
    const graphRes = await fetch('./assets/graph.json');
    if (!graphRes.ok) throw new Error(`failed to fetch graph.json: ${graphRes.status}`);
    graph = await graphRes.json();
  }

  const reportData = readEmbeddedJson('report-data') || {};
  const reportConfig = readEmbeddedJson('report-config') || {};

  renderSidebar(sidebarEl, reportData, reportConfig);

  const elements = [];

  for (const n of graph.nodes) {
    elements.push({
      data: {
        id: n.id,
        label: n.label,
        kind: n.kind,
        has_findings: !!n.has_findings,
      },
    });
  }

  for (const e of graph.edges) {
    elements.push({
      data: {
        id: `${e.source} -> ${e.target}`,
        source: e.source,
        target: e.target,
        dep_type: e.dep_type || 'build',
      },
    });
  }

  const cy = cytoscape({
    container: document.getElementById('graph'),
    elements,
    style: [
      {
        selector: 'node',
        style: {
          'label': 'data(label)',
          'font-size': 8,
          'text-wrap': 'wrap',
          'text-max-width': 120,
          'background-color': '#4f7bd9',
          'color': '#111',
          'border-width': 0,
          'width': 10,
          'height': 10,
        },
      },
      {
        selector: 'node[kind = "external"]',
        style: {
          'background-color': '#bdbdbd',
        },
      },
      {
        selector: 'node[has_findings]',
        style: {
          'border-width': 2,
          'border-color': '#d9534f',
        },
      },
      {
        selector: 'edge',
        style: {
          'width': 1,
          'line-color': '#999',
          'target-arrow-color': '#999',
          'target-arrow-shape': 'triangle',
          'curve-style': 'bezier',
          'arrow-scale': 0.8,
        },
      },
      {
        selector: 'edge[dep_type = "build"]',
        style: {
          'line-color': '#000',
          'target-arrow-color': '#000',
        },
      },
      {
        selector: 'edge[dep_type = "exec"]',
        style: {
          'line-color': '#2e7d32',
          'target-arrow-color': '#2e7d32',
        },
      },
      {
        selector: 'edge[dep_type = "test"]',
        style: {
          'line-color': '#ef6c00',
          'target-arrow-color': '#ef6c00',
        },
      },
    ],
    layout: {
      name: 'breadthfirst',
      directed: true,
      padding: 20,
      spacingFactor: 1.1,
    },
    wheelSensitivity: 0.15,
  });

  function applyFilters() {
    const showExternal = !!showExternalEl?.checked;
    const showNoFindings = !!showNoFindingsEl?.checked;

    cy.batch(() => {
      cy.nodes().forEach((n) => {
        const kind = n.data('kind');
        const hasFindings = !!n.data('has_findings');

        let visible = true;
        if (kind === 'external' && !showExternal) visible = false;
        if (kind !== 'external' && !hasFindings && !showNoFindings) visible = false;

        if (visible) n.show();
        else n.hide();
      });
    });

    const visibleNodes = cy.nodes(':visible').length;
    const visibleEdges = cy.edges(':visible').length;
    setStatus(`Visible: ${visibleNodes} nodes, ${visibleEdges} edges`);
  }

  showExternalEl?.addEventListener('change', applyFilters);
  showNoFindingsEl?.addEventListener('change', applyFilters);

  applyFilters();

  setStatus(`Loaded: ${cy.nodes().length} nodes, ${cy.edges().length} edges`);
}

function readEmbeddedJson(id) {
  const el = document.getElementById(id);
  if (!el || !el.textContent) return null;
  const txt = el.textContent.trim();
  if (!txt) return null;
  return JSON.parse(txt);
}

function renderSidebar(sidebarEl, reportData, reportConfig) {
  if (!sidebarEl) return;

  const sections = Array.isArray(reportConfig.sections)
    ? reportConfig.sections
    : ['summary', 'overview', 'packages', 'package_findings', 'external_deps', 'legend'];
  const hidden = new Set(Array.isArray(reportConfig.hidden) ? reportConfig.hidden : []);

  sidebarEl.innerHTML = '';

  for (const sectionId of sections) {
    if (hidden.has(sectionId)) continue;

    if (sectionId === 'summary') {
      sidebarEl.appendChild(renderSummarySection(reportData.summary || []));
    } else if (sectionId === 'overview') {
      sidebarEl.appendChild(renderOverviewSection(reportData.overview || {}));
    } else if (sectionId === 'packages') {
      sidebarEl.appendChild(renderPackagesSection(reportData.packages || {}));
    } else if (sectionId === 'package_findings') {
      sidebarEl.appendChild(renderPackageFindingsSection(reportData.packages || {}));
    } else if (sectionId === 'external_deps') {
      sidebarEl.appendChild(renderExternalDepsSection(reportData.external_deps || {}));
    } else if (sectionId === 'legend') {
      sidebarEl.appendChild(renderLegendSection());
    }
  }
}

function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') node.className = v;
    else if (k === 'text') node.textContent = v;
    else node.setAttribute(k, v);
  }
  for (const c of children) node.appendChild(c);
  return node;
}

function renderOverviewSection(overview) {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'Overview' }));

  const kv = el('div', { class: 'kv' });
  kv.appendChild(el('div', { text: 'Packages' }));
  kv.appendChild(el('div', { text: String(overview.total_packages ?? '-') }));
  kv.appendChild(el('div', { text: 'Findings' }));
  kv.appendChild(el('div', { text: String(overview.total_findings ?? '-') }));
  if (overview.source_root) {
    kv.appendChild(el('div', { text: 'Source root' }));
    kv.appendChild(el('div', { text: overview.source_root }));
  }
  section.appendChild(kv);

  const groups = Array.isArray(overview.top_groups) ? overview.top_groups : [];
  if (groups.length > 0) {
    section.appendChild(el('div', { class: 'muted', text: 'Top groups (by packages)' }));
    const table = el('table', { class: 'table' });
    table.appendChild(el('thead', {}, [
      el('tr', {}, [
        el('th', { text: 'Group' }),
        el('th', { text: 'Packages' }),
        el('th', { text: 'With findings' }),
      ]),
    ]));
    const tbody = el('tbody');
    for (const g of groups) {
      tbody.appendChild(el('tr', {}, [
        el('td', { text: g.name || '' }),
        el('td', { text: String(g.packages ?? 0) }),
        el('td', { text: String(g.packages_with_findings ?? 0) }),
      ]));
    }
    table.appendChild(tbody);
    section.appendChild(table);
  }

  return section;
}

function renderSummarySection(summaryItems) {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'Summary' }));

  const items = Array.isArray(summaryItems) ? summaryItems : [];
  if (items.length === 0) {
    section.appendChild(el('div', { class: 'muted', text: 'No summary data.' }));
    return section;
  }

  const table = el('table', { class: 'table' });
  table.appendChild(el('thead', {}, [
    el('tr', {}, [
      el('th', { text: 'Key' }),
      el('th', { text: 'Value' }),
    ]),
  ]));
  const tbody = el('tbody');
  for (const it of items) {
    tbody.appendChild(el('tr', {}, [
      el('td', { text: String(it.key ?? '') }),
      el('td', { text: String(it.value ?? 0) }),
    ]));
  }
  table.appendChild(tbody);
  section.appendChild(table);
  return section;
}

function renderPackagesSection(packages) {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'Packages' }));

  const itemsRaw = Array.isArray(packages.items) ? packages.items : [];
  if (itemsRaw.length === 0) {
    section.appendChild(el('div', { class: 'muted', text: 'No packages found.' }));
    return section;
  }

  const items = itemsRaw.slice().sort((a, b) => String(a.name || '').localeCompare(String(b.name || '')));

  const table = el('table', { class: 'table' });
  table.appendChild(el('thead', {}, [
    el('tr', {}, [
      el('th', { text: 'Name' }),
      el('th', { text: 'Path' }),
    ]),
  ]));
  const tbody = el('tbody');
  for (const it of items) {
    tbody.appendChild(el('tr', {}, [
      el('td', { text: String(it.name ?? '') }),
      el('td', { text: String(it.path ?? '') }),
    ]));
  }
  table.appendChild(tbody);
  section.appendChild(table);
  return section;
}

function renderPackageFindingsSection(packages) {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'Package findings' }));

  const items = Array.isArray(packages.items) ? packages.items : [];
  if (items.length === 0) {
    section.appendChild(el('div', { class: 'muted', text: 'No packages found.' }));
    return section;
  }

  section.appendChild(el('div', { class: 'muted', text: 'Sorted by findings count (desc).' }));

  const table = el('table', { class: 'table' });
  table.appendChild(el('thead', {}, [
    el('tr', {}, [
      el('th', { text: 'Package' }),
      el('th', { text: 'Findings' }),
      el('th', { text: 'Has findings' }),
    ]),
  ]));
  const tbody = el('tbody');
  for (const it of items) {
    tbody.appendChild(el('tr', {}, [
      el('td', { text: String(it.name ?? '') }),
      el('td', { text: String(it.findings_count ?? 0) }),
      el('td', { text: (it.has_findings ? 'yes' : 'no') }),
    ]));
  }
  table.appendChild(tbody);
  section.appendChild(table);
  return section;
}

function renderExternalDepsSection(externalDeps) {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'External deps' }));

  const items = Array.isArray(externalDeps.top) ? externalDeps.top : [];
  if (items.length === 0) {
    section.appendChild(el('div', { class: 'muted', text: 'No external dependencies found.' }));
    return section;
  }

  section.appendChild(el('div', { class: 'muted', text: 'Usage count = number of workspace packages that depend on it' }));

  const table = el('table', { class: 'table' });
  table.appendChild(el('thead', {}, [
    el('tr', {}, [
      el('th', { text: 'Name' }),
      el('th', { text: 'Count' }),
    ]),
  ]));
  const tbody = el('tbody');
  for (const it of items) {
    tbody.appendChild(el('tr', {}, [
      el('td', { text: it.name || '' }),
      el('td', { text: String(it.count ?? 0) }),
    ]));
  }
  table.appendChild(tbody);
  section.appendChild(table);
  return section;
}

function renderLegendSection() {
  const section = el('section', { class: 'section' });
  section.appendChild(el('h2', { text: 'Legend' }));
  section.appendChild(el('div', { class: 'muted', text: 'Edges: build=black, exec=green, test=orange' }));
  return section;
}

main().catch((e) => {
  const statusEl = document.getElementById('status');
  if (statusEl) {
    statusEl.textContent = `Error: ${e.message} (If you opened index.html via file:// and see this, the embedded graph data might be missing.)`;
  }
  console.error(e);
});
