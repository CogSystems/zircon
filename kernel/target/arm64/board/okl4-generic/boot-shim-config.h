/*
 * Copyright 2018 Cog Systems Pty Ltd. All rights reserved.
 *
 * Use of this source code is governed by the 3-Clause BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <libfdt.h>

#ifndef fdt_for_each_subnode
#define fdt_for_each_subnode(node, fdt, parent)		\
	for (node = fdt_first_subnode(fdt, parent);	\
	     node >= 0;					\
	     node = fdt_next_subnode(fdt, node))
#endif

#ifndef fdt_for_each_compatible_node
#define fdt_for_each_compatible_node(node, fdt, compatible) \
    for (int node = fdt_node_offset_by_compatible(fdt, -1, compatible); \
            node >= 0; \
            node = fdt_node_offset_by_compatible(fdt, node, compatible))
#endif

#define HAS_DEVICE_TREE 1

/* allow space for two clusters but default to uniprocessor */
static int max_clusters = 2;
static zbi_cpu_config_t cpu_config = {
    .cluster_count = 1,
    .clusters = {
        {
            .cpu_count = 1,
        },
        {
            .cpu_count = 0,
        },
    },
};

static int num_mem_configs;
static zbi_mem_range_t mem_config[16];
static dcfg_simple_t uart_driver;
static dcfg_arm_gicv2_driver_t gicv2_driver = {
    .gicd_offset = 0x1000,
    .gicc_offset = 0x2000,
    .ipi_base = 2,
};
static dcfg_arm_psci_driver_t psci_driver = {
    .use_hvc = true
};
static dcfg_arm_generic_timer_driver_t timer_driver;
static dcfg_hyp_vtty_driver_t hyp_vtty = {
    .tx_irq = ~0U,
    .rx_irq = ~0U,
    .tx_kcap = ~0U,
    .rx_kcap = ~0U,
};

static const zbi_platform_id_t platform_id = {
    .vid = PDEV_VID_OKL4,
    .pid = PDEV_PID_OKL4,
    .board_name = "okl4-generic",
};

static int num_link_shbufs;
static link_shbuf_info_t link_shbuf_info[32];

static int num_vs_shbufs;
static vs_shbuf_info_t vs_shbuf_info[32];

static int num_link_pipes;
static link_pipe_info_t link_pipe_info[32];

static struct {
    const char *cmdline;
    size_t cmdline_length;
    uintptr_t initrd_start;
} chosen;

static void append_board_boot_item(zbi_header_t* bootdata)
{
    // add CPU configuration
    append_boot_item(bootdata, ZBI_TYPE_CPU_CONFIG, 0, &cpu_config,
                     sizeof(zbi_cpu_config_t) +
                     sizeof(zbi_cpu_cluster_t) * cpu_config.cluster_count);

    // add memory configuration
    append_boot_item(bootdata, ZBI_TYPE_MEM_CONFIG, 0, &mem_config,
                     sizeof(zbi_mem_range_t) * num_mem_configs);

    // add kernel drivers
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_GIC_V2,
                     &gicv2_driver, sizeof(gicv2_driver));
    if (uart_driver.irq != 0)
        append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_PL011_UART,
                         &uart_driver, sizeof(uart_driver));
    if (hyp_vtty.tx_kcap != ~0U && hyp_vtty.rx_kcap != ~0U &&
            hyp_vtty.tx_irq != ~0U && hyp_vtty.rx_irq != ~0U) {
        append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_HYP_VTTY,
                         &hyp_vtty, sizeof(hyp_vtty));
    }
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_PSCI,
                     &psci_driver, sizeof(psci_driver));
    if (timer_driver.irq_virt != 0)
        append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER,
                         KDRV_ARM_GENERIC_TIMER, &timer_driver,
                         sizeof(timer_driver));

    // append kernel command line
    if (chosen.cmdline && chosen.cmdline_length) {
        append_boot_item(bootdata, ZBI_TYPE_CMDLINE, 0,
                         chosen.cmdline, chosen.cmdline_length);
    }

    // add metadata
    for (int i = 0; i < num_link_shbufs; i++)
        append_boot_item(bootdata, LINK_SHBUF_METADATA, i,
                &link_shbuf_info[i], sizeof(link_shbuf_info[i]));
    for (int i = 0; i < num_link_pipes; i++)
        append_boot_item(bootdata, LINK_PIPE_METADATA, i,
                &link_pipe_info[i], sizeof(link_pipe_info[i]));
    for (int i = 0; i < num_vs_shbufs; i++)
        append_boot_item(bootdata, VS_SHBUF_METADATA, i,
                &vs_shbuf_info[i], sizeof(vs_shbuf_info[i]));

    // add platform ID
    append_boot_item(bootdata, ZBI_TYPE_PLATFORM_ID, 0, &platform_id,
                     sizeof(platform_id));
}

static int count_cpu_nodes(void *device_tree)
{
    int num_cpus = 0;
    int offset = -1;
    while (1) {
        offset = fdt_node_offset_by_prop_value(device_tree, offset,
                                               "device_type", "cpu", 4);
        if (offset < 0)
            break;
        num_cpus++;
    }

    return num_cpus;
}

static int cluster_cpu_count(void *device_tree, int cluster_num)
{
    /*
        cpu-map {
            cluster0 {
                core0 {
                    cpu = <&CPU0>;
                };
                core1 {
                    cpu = <&CPU1>;
                };
                core2 {
                    cpu = <&CPU2>;
                };
                core3 {
                    cpu = <&CPU3>;
                };
            };

            cluster1 {
                core0 {
                    cpu = <&CPU4>;
                };
                core1 {
                    cpu = <&CPU5>;
                };
                core2 {
                    cpu = <&CPU6>;
                };
                core3 {
                    cpu = <&CPU7>;
                };
            };
        };
    */
    char cluster_node[] = "/cpus/cpu-map/clusterXX";
    if (cluster_num > 10) {
        cluster_node[sizeof(cluster_node) - 3] = cluster_num / 10 + '0';
        cluster_node[sizeof(cluster_node) - 2] = cluster_num % 10 + '0';
    } else {
        cluster_node[sizeof(cluster_node) - 3] = cluster_num + '0';
        cluster_node[sizeof(cluster_node) - 2] = '\0';
    }
    int offset = fdt_path_offset(device_tree, cluster_node);
    if (offset < 0) {
        /*
         * If there's no cpu-map cluster node and this is the first cluster,
         * just count the total number of cpu nodes instead and assume it's
         * a single cluster.
         */
        if (cluster_num == 0)
            return count_cpu_nodes(device_tree);
        return offset;
    }

    int cpu_num = 0;
    char cpu_node[] = "coreXX";
    while (1) {
        if (cpu_num > 10) {
            cpu_node[sizeof(cpu_node) - 3] = cpu_num / 10 + '0';
            cpu_node[sizeof(cpu_node) - 2] = cpu_num % 10 + '0';
        } else {
            cpu_node[sizeof(cpu_node) - 3] = cpu_num + '0';
            cpu_node[sizeof(cpu_node) - 2] = '\0';
        }
        if (fdt_subnode_offset(device_tree, offset, cpu_node) < 0)
            break;
        cpu_num++;
    }

    return cpu_num;
}

static void add_memory_range(uint32_t type, uint64_t address,
                             uint64_t size)
{
    if (num_mem_configs == countof(mem_config))
        fail("too many memory regions\n");
    mem_config[num_mem_configs].type = type;
    mem_config[num_mem_configs].paddr = address;
    mem_config[num_mem_configs].length = size;
    num_mem_configs++;
}

static void add_link_shbuf(const char *name, uint32_t type, uint64_t address,
                             uint64_t size, uint32_t virqline, uint32_t virq,
                             uint32_t rwx)
{
    if (num_link_shbufs == countof(link_shbuf_info))
        fail("too many shared buffers\n");
    strlcpy(link_shbuf_info[num_link_shbufs].name, name,
            sizeof(link_shbuf_info[num_link_shbufs].name));
    link_shbuf_info[num_link_shbufs].type = type;
    link_shbuf_info[num_link_shbufs].paddr = address;
    link_shbuf_info[num_link_shbufs].size = size;
    link_shbuf_info[num_link_shbufs].virqline = virqline;
    link_shbuf_info[num_link_shbufs].virq = virq;
    link_shbuf_info[num_link_shbufs].rwx = rwx;
    num_link_shbufs++;
}

static void add_vs_shbuf(vs_shbuf_info_t *info)
{
    if (num_vs_shbufs == countof(vs_shbuf_info))
        fail("too many shared buffers\n");
    memcpy(&vs_shbuf_info[num_vs_shbufs], info, sizeof(vs_shbuf_info[0]));
    num_vs_shbufs++;
}

static void add_link_pipe(const char *name, uint32_t tx_irq, uint32_t rx_irq,
                          uint32_t tx_kcap, uint32_t rx_kcap)
{
    if (num_link_pipes == countof(link_pipe_info))
        fail("too many pipes\n");
    strlcpy(link_pipe_info[num_link_pipes].name, name,
            sizeof(link_pipe_info[num_link_pipes].name));
    link_pipe_info[num_link_pipes].tx_irq = tx_irq;
    link_pipe_info[num_link_pipes].rx_irq = rx_irq;
    link_pipe_info[num_link_pipes].tx_kcap = tx_kcap;
    link_pipe_info[num_link_pipes].rx_kcap = rx_kcap;
    num_link_pipes++;
}

// Parse the device tree to find our ZBI, kernel command line, and RAM size.
static void *read_device_tree(void* device_tree)
{
    int parent, offset, length, address_cells, size_cells;
    const void *property;
    const uint32_t *data32;
    const uint64_t *data64;
    uint64_t address, size;

    if (fdt_check_header(device_tree) < 0) {
        fail("fdt_check_header failed\n");
    }

    /* cpu count + clusters */
    int cluster_num = 0;
    while (1) {
        int cpu_count = cluster_cpu_count(device_tree, cluster_num);
        if (cpu_count <= 0)
            break;
        if (cluster_num == max_clusters)
            fail("too many cpu clusters\n");
        cpu_config.clusters[cluster_num].cpu_count = cpu_count;
        cluster_num++;
    }
    cpu_config.cluster_count = cluster_num;

    /* /memreserve/ */
    int count = fdt_num_mem_rsv(device_tree);
    for (int i = 0; i < count; i++) {
        if (fdt_get_mem_rsv(device_tree, i, &address, &size) < 0)
            break;
        add_memory_range(ZBI_MEM_RANGE_RESERVED, address, size);
    }

    /* memory */
    /*
        memory@20200000 {
            device_type = "memory";
            reg = <0x0 0x20200000 0x10000000>;
        };
    */
    offset = -1;
    while (1) {
        offset = fdt_node_offset_by_prop_value(device_tree, offset,
                                               "device_type", "memory",
                                               7);
        if (offset < 0)
            break;
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (!property)
            continue;
        data64 = property;
        data32 = property;
        if (length == sizeof(uint32_t) * 4)
            size = fdt64_to_cpu(data64[1]);
        else if (length == sizeof(uint32_t) * 3)
            size = fdt32_to_cpu(data32[2]);
        else
            fail("invalid memory node\n");
        address = fdt64_to_cpu(data64[0]);
        add_memory_range(ZBI_MEM_RANGE_RAM, address, size);
    }

    /* command line */
    offset = fdt_path_offset(device_tree, "/chosen");
    if (offset >= 0) {
        chosen.cmdline = fdt_getprop(device_tree, offset, "bootargs", &length);
        chosen.cmdline_length = length;
        property = fdt_getprop(device_tree, offset, "linux,initrd-start",
                &length);
        if (property) {
            if (length == sizeof(uint64_t)) {
                data64 = property;
                chosen.initrd_start = fdt64_to_cpu(*data64);
            } else {
                data32 = property;
                chosen.initrd_start = fdt32_to_cpu(*data32);
            }
        }
    }

    /* pipe console */
    /*
		serial@8 {
			compatible = "okl,pipe-tty", "okl,microvisor-pipe", "okl,microvisor-capability";
			phandle = <0x6>;
			reg = <0x8 0x9>;
			label = "serial";
			interrupts = <0x0 0x0 0x1 0x0 0x1 0x1>;
		};
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "okl,pipe-tty");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (property && length == sizeof(uint32_t) * 2) {
            data32 = property;
            hyp_vtty.tx_kcap = fdt32_to_cpu(data32[0]);
            hyp_vtty.rx_kcap = fdt32_to_cpu(data32[1]);
        }
        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (property && length == sizeof(uint32_t) * 6) {
            data32 = property;
            hyp_vtty.tx_irq = fdt32_to_cpu(data32[1])
                              + (fdt32_to_cpu(data32[0]) ? 16 : 32);
            hyp_vtty.rx_irq = fdt32_to_cpu(data32[4])
                              + (fdt32_to_cpu(data32[3]) ? 16 : 32);
        }
    }

    /* hardware uart */
    /*
        uart@f7113000 {
            compatible = "arm,pl011";
            reg = <0x0 0xf7113000 0x1000>;
            interrupts = <0x0 0x27 0x4>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,pl011");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (property) {
            parent = fdt_parent_offset(device_tree, offset);
            address_cells = fdt_address_cells(device_tree, parent);
            if (address_cells == 1) {
                data32 = property;
                uart_driver.mmio_phys = fdt32_to_cpu(*data32);
            } else {
                data64 = property;
                uart_driver.mmio_phys = fdt64_to_cpu(*data64);
            }
            add_memory_range(ZBI_MEM_RANGE_PERIPHERAL,
                             uart_driver.mmio_phys, 0x1000);
        }
        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (property && length == sizeof(uint32_t) * 3) {
            data32 = property;
            uart_driver.irq = fdt32_to_cpu(data32[1])
                              + (fdt32_to_cpu(data32[0]) ? 16 : 32);
        }
    }

    /* psci method */
    /*
        psci {
            compatible = "arm,psci-1.1", "arm,psci-1.0", "arm,psci-0.2", "arm,psci-0.1", "arm,psci";
            method = "hvc";
            cpu_suspend = <0xc4000001>;
            cpu_off = <0x84000002>;
            cpu_on = <0xc4000003>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,psci");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "method", &length);
        if (property)
            psci_driver.use_hvc = !strcmp(property, "hvc");
    }

    /* timer */
    /*
        timer {
            compatible = "okl,virtual-arm-timer", "arm,armv8-timer", "arm,armv7-timer";
            always-on;
            interrupts = <0x1 0x0 0x4 0x1 0x0 0x4 0x1 0xb 0x104 0x1 0x0 0x4>;
            interrupt-parent = <&intc>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "okl,virtual-arm-timer");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,armv8-timer");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,armv7-timer");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (property && length == sizeof(uint32_t) * 12) {
            data32 = property;
            timer_driver.irq_virt = fdt32_to_cpu(data32[7])
                                    + (fdt32_to_cpu(data32[6]) ? 16 : 32);
        }
    }

    /* gic */
    /*
        interrupt-controller {
            compatible = "okl,hypervisor-vgic", "arm,cortex-a9-gic", "arm,pl390";
            interrupt-controller;
            #interrupt-cells = <0x3>;
            phandle = <0x3>;
            interrupt-parent = <0x3>;
            reg = <0x0 0xf6801000 0x1000 0x0 0xf6802000 0x1000>;
        };
    */
    offset = fdt_node_offset_by_compatible(device_tree, -1, "okl,hypervisor-vgic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,cortex-a15-gic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,cortex-a9-gic");
    if (offset < 0)
        offset = fdt_node_offset_by_compatible(device_tree, -1, "arm,pl390");
    if (offset >= 0) {
        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (property) {
            uint64_t gicc_address, gicc_size, gicd_address, gicd_size;
            parent = fdt_parent_offset(device_tree, offset);
            address_cells = fdt_address_cells(device_tree, parent);
            size_cells = fdt_address_cells(device_tree, parent);
            if (address_cells == 1) {
                data32 = property;
                gicd_address = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicd_address = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (size_cells == 1) {
                data32 = property;
                gicd_size = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicd_size = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (address_cells == 1) {
                data32 = property;
                gicc_address = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicc_address = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (size_cells == 1) {
                data32 = property;
                gicc_size = fdt32_to_cpu(*data32);
                property = &data32[1];
            } else {
                data64 = property;
                gicc_size = fdt64_to_cpu(*data64);
                property = &data64[1];
            }
            if (gicd_address < gicc_address)
                gicv2_driver.mmio_phys = gicd_address;
            else
                gicv2_driver.mmio_phys = gicc_address;
            /*
             * Zircon's GICv2 crashes during initialisation if we just map
             * 4kB each for the GICD and GICC.  As a workaround, we map a
             * 64kB region and assume that both are in that region.  This is
             * true for a real GICv2 and we can control the addresses of the
             * virtual GICv2 to ensure that it's also true for the virtual
             * GICv2.
             */
            gicv2_driver.mmio_phys &= ~(uint64_t)0xfffUL;
            gicv2_driver.gicd_offset = gicd_address - gicv2_driver.mmio_phys;
            gicv2_driver.gicc_offset = gicc_address - gicv2_driver.mmio_phys;
            add_memory_range(ZBI_MEM_RANGE_PERIPHERAL, gicv2_driver.mmio_phys,
                             0x10000);
        }
    }
    if (offset < 0 || !property)
        fail("gic not found in device tree\n");

    /* shared buffers */
    /*
        linux-shbuf@10181000 {
            compatible = "okl,microvisor-link-shbuf", "zircon,block", "okl,microvisor-shared-memory";
            phandle = <0xe>;
            reg = <0x0 0x10181000 0x1000>;
            label = "linux-shbuf";
            okl,rwx = <0x6>;
            okl,interrupt-line = <0xa>;
            interrupts = <0x0 0x3 0x1>;
            interrupt-parent = <0x3>;
        };

		interrupt-line@f {
			compatible = "okl,microvisor-interrupt-line", "okl,microvisor-capability";
			phandle = <0xa>;
			reg = <0xf>;
			label = "linux-shbuf_virqline";
		};
    */
    uint32_t type, virq, virqline, rwx;
    int virq_offset;
    const char *label;

    fdt_for_each_compatible_node(offset, device_tree,
                                 "okl,microvisor-link-shbuf") {
        if (!fdt_node_check_compatible(device_tree, offset, "zircon,block"))
            type =  LINK_SHBUF_TYPE_BLOCK;
        else if (!fdt_node_check_compatible(device_tree, offset, "zircon,test"))
            type =  LINK_SHBUF_TYPE_TEST;
        else
            type =  LINK_SHBUF_TYPE_GENERIC;

        property = fdt_getprop(device_tree, offset, "label", &length);
        if (!property)
            continue;
		label = property;

        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (!property)
            continue;
        parent = fdt_parent_offset(device_tree, offset);
        address_cells = fdt_address_cells(device_tree, parent);
        size_cells = fdt_address_cells(device_tree, parent);
        if (address_cells == 1) {
            data32 = property;
            address = fdt32_to_cpu(data32[0]);
            property = &data32[1];
        } else {
            data64 = property;
            address = fdt64_to_cpu(data64[0]);
            property = &data64[1];
        }
        if (size_cells == 2) {
            data64 = property;
            size = fdt64_to_cpu(data64[0]);
        } else {
            data32 = property;
            size = fdt32_to_cpu(data32[0]);
        }

        property = fdt_getprop(device_tree, offset, "okl,rwx", &length);
        if (!property)
            continue;
        data32 = property;
		rwx = fdt32_to_cpu(data32[0]);

        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (!property)
            continue;
        data32 = property;
        virq = fdt32_to_cpu(data32[1]) + (fdt32_to_cpu(data32[0]) ? 16 : 32);

        property = fdt_getprop(device_tree, offset, "okl,interrupt-line",
                               &length);
        if (!property)
            continue;
        data32 = property;

        virq_offset = fdt_node_offset_by_phandle(device_tree,
                                                 fdt32_to_cpu(data32[0]));
        if (virq_offset < 0)
            continue;
        if (fdt_node_check_compatible(device_tree, virq_offset,
                                      "okl,microvisor-interrupt-line"))
            continue;
        property = fdt_getprop(device_tree, virq_offset, "reg", &length);
        if (!property)
            continue;
        data32 = property;
        virqline = fdt32_to_cpu(data32[0]);

        add_link_shbuf(label, type, address, size, virqline, virq, rwx);
    }

    /* virtual service session shared buffer transport */
    /*
        shared-buffer@40000 {
            compatible = "okl,microvisor-shared-memory-transport", "okl,microvisor-shared-memory";
            phandle = <0x10>;
            reg = <0x0 0x40000 0x40000 0x0 0x0 0x40000>;
            label = "client";
            okl,rwx = <0x6 0x6>;

            virtual-session {
                compatible = "okl,virtual-session";
                label = "client";
                #address-cells = <0x1>;
                #size-cells = <0x0>;
                okl,is-client;
                okl,notify-bits = <0x0 0x1>;
                interrupts = <0x0 0x4 0x1>;
                interrupt-parent = <0x3>;
                okl,interrupt-lines = <0xd>;
                okl,queue-length = <0x10>;
                okl,message-size = <0x3ffe>;
                okl,batch-size = <0x10 0x10>;
            };

		interrupt-line@16 {
			compatible = "okl,microvisor-interrupt-line", "okl,microvisor-capability";
			phandle = <0xd>;
			reg = <0x16>;
			label = "linux2_client_out_irq_virqline";
		};
    */

    fdt_for_each_compatible_node(offset, device_tree,
                                 "okl,microvisor-shared-memory-transport") {
        int vs_offset;

        parent = fdt_parent_offset(device_tree, offset);
        address_cells = fdt_address_cells(device_tree, parent);
        size_cells = fdt_address_cells(device_tree, parent);

        fdt_for_each_subnode(vs_offset, device_tree, offset) {
            if (fdt_node_check_compatible(device_tree, vs_offset,
                                          "okl,virtual-session"))
                continue;

            vs_shbuf_info_t info;

            if (fdt_getprop(device_tree, vs_offset, "okl,is-client", NULL))
                info.is_client = true;
            else if (fdt_getprop(device_tree, vs_offset, "okl,is-server", NULL))
                info.is_client = false;
            else
                continue;
            property = fdt_getprop(device_tree, offset, "label", &length);
            if (!property)
                continue;
            strlcpy(info.name, property, sizeof(info.name));

            property = fdt_getprop(device_tree, offset, "reg", &length);
            if (!property)
                continue;
            if (address_cells == 1) {
                data32 = property;
                info.tx_buf.paddr = fdt32_to_cpu(data32[0]);
                property = &data32[1];
            } else {
                data64 = property;
                info.tx_buf.paddr = fdt64_to_cpu(data64[0]);
                property = &data64[1];
            }
            if (size_cells == 2) {
                data64 = property;
                info.tx_buf.size = fdt64_to_cpu(data64[0]);
                property = &data64[1];
            } else {
                data32 = property;
                info.tx_buf.size = fdt32_to_cpu(data32[0]);
                property = &data32[1];
            }
            if (address_cells == 1) {
                data32 = property;
                info.rx_buf.paddr = fdt32_to_cpu(data32[0]);
                property = &data32[1];
            } else {
                data64 = property;
                info.rx_buf.paddr = fdt64_to_cpu(data64[0]);
                property = &data64[1];
            }
            if (size_cells == 2) {
                data64 = property;
                info.rx_buf.size = fdt64_to_cpu(data64[0]);
                property = &data64[1];
            } else {
                data32 = property;
                info.rx_buf.size = fdt32_to_cpu(data32[0]);
                property = &data32[1];
            }

            property = fdt_getprop(device_tree, offset, "okl,rwx", &length);
            if (!property)
                continue;
            data32 = property;
            info.tx_buf.rwx = fdt32_to_cpu(data32[0]);
            info.rx_buf.rwx = fdt32_to_cpu(data32[1]);

            property = fdt_getprop(device_tree, vs_offset, "okl,notify-bits", &length);
            if (!property)
                continue;
            data32 = property;
            info.tx_buf.notify_bits = fdt32_to_cpu(data32[0]);
            info.rx_buf.notify_bits = fdt32_to_cpu(data32[1]);

            property = fdt_getprop(device_tree, vs_offset, "okl,queue-length", &length);
            if (!property)
                continue;
            data32 = property;
            info.queue_length = fdt32_to_cpu(data32[0]);

            property = fdt_getprop(device_tree, vs_offset, "okl,message-size", &length);
            if (!property)
                continue;
            data32 = property;
            info.message_size = fdt32_to_cpu(data32[0]);

            property = fdt_getprop(device_tree, vs_offset, "okl,batch-size", &length);
            if (!property)
                continue;
            data32 = property;
            info.tx_buf.batch_size = fdt32_to_cpu(data32[0]);
            info.rx_buf.batch_size = fdt32_to_cpu(data32[1]);

            property = fdt_getprop(device_tree, vs_offset, "interrupts", &length);
            if (!property)
                continue;
            data32 = property;
            info.virq = fdt32_to_cpu(data32[1]) + (fdt32_to_cpu(data32[0]) ? 16 : 32);

            property = fdt_getprop(device_tree, vs_offset, "okl,interrupt-lines",
                                   &length);
            if (!property)
                continue;
            data32 = property;

            virq_offset = fdt_node_offset_by_phandle(device_tree,
                                                     fdt32_to_cpu(data32[0]));
            if (virq_offset < 0)
                continue;
            if (fdt_node_check_compatible(device_tree, virq_offset,
                                          "okl,microvisor-interrupt-line"))
                continue;
            property = fdt_getprop(device_tree, virq_offset, "reg", &length);
            if (!property)
                continue;
            data32 = property;
            info.virqline = fdt32_to_cpu(data32[0]);

            add_vs_shbuf(&info);
            break;
        }
    }

    /* pipes */
    /*
        aliases {
            pipe0 = "/hypervisor/pipe0@18";
        };

		pipe0@18 {
			compatible = "okl,pipe", "okl,microvisor-pipe", "okl,microvisor-capability";
			phandle = <0xb>;
			reg = <0x18 0x19>;
			label = "pipe0";
			interrupts = <0x0 0xa 0x1 0x0 0xb 0x1>;
		};
    */
    fdt_for_each_compatible_node(offset, device_tree, "okl,pipe") {
        uint32_t tx_irq, rx_irq, tx_kcap, rx_kcap;

        property = fdt_getprop(device_tree, offset, "label", &length);
        if (!property)
            continue;
		label = property;

        property = fdt_getprop(device_tree, offset, "reg", &length);
        if (!property || length != sizeof(uint32_t) * 2)
            continue;
        data32 = property;
        tx_kcap = fdt32_to_cpu(data32[0]);
        rx_kcap = fdt32_to_cpu(data32[1]);

        property = fdt_getprop(device_tree, offset, "interrupts", &length);
        if (!property || length != sizeof(uint32_t) * 6)
            continue;
        data32 = property;
        tx_irq = fdt32_to_cpu(data32[1])
                 + (fdt32_to_cpu(data32[0]) ? 16 : 32);
        rx_irq = fdt32_to_cpu(data32[4])
                 + (fdt32_to_cpu(data32[3]) ? 16 : 32);

        add_link_pipe(label, tx_irq, rx_irq, tx_kcap, rx_kcap);
    }

    // Use the device tree initrd as the ZBI.
    return (void*)chosen.initrd_start;
}
