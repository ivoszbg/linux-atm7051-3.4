#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <mach/clock.h>


static DEFINE_MUTEX(asoc_clock_lock);
static LIST_HEAD(asoc_clocks);
static LIST_HEAD(root_clks);
extern struct clk_ops leopard_clk_shared_bus_ops;

void clk_init(struct clk *c)
{
    if(!c)
    {
        return;
    }
	spin_lock_init(&c->spinlock);
	INIT_LIST_HEAD(&c->children);	

	if (c->ops && c->ops->init)
		c->ops->init(c);
	if (!c->ops || !c->ops->enable) {
	    c->refcnt++;
		c->set = true;
		if (c->parent)
			c->state = c->parent->state;
		else
			c->state = ON;
	}
	
	mutex_lock(&asoc_clock_lock);
	if (c->parent)
	{
		list_add(&c->sibling, &c->parent->children);
	}
	else
	{
		list_add(&c->sibling, &root_clks);
	}
	list_add(&c->list, &asoc_clocks);
	mutex_unlock(&asoc_clock_lock);
}

int clk_enable(struct clk *c)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&c->spinlock, flags);
	if (c->refcnt == 0) {
		if (c->parent) {
			ret = clk_enable(c->parent);
			if (ret)
				goto out;
		}

		if (c->ops && c->ops->enable) {
			ret = c->ops->enable(c);
			if (ret) {
				if (c->parent)
					clk_disable(c->parent);
				goto out;
			}
			c->state = ON;
			c->set = true;
		}
	}
	c->refcnt++;
out:
	spin_unlock_irqrestore(&c->spinlock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_reset(struct clk *c)
{
    if(!c || !c->ops || !c->ops->reset)
    {
        return;
    }
    c->ops->reset(c);
}
EXPORT_SYMBOL(clk_reset);

void clk_disable(struct clk *c)
{
	unsigned long flags;

	spin_lock_irqsave(&c->spinlock, flags);

	if (c->refcnt == 0) {
		WARN(1, "Attempting to disable clock %s with refcnt 0", c->name);
		spin_unlock_irqrestore(&c->spinlock, flags);
		return;
	}
	if (c->refcnt == 1) {
		if (c->ops && c->ops->disable)
			c->ops->disable(c);

		if (c->parent)
			clk_disable(c->parent);

		c->state = OFF;
	}
	c->refcnt--;

	spin_unlock_irqrestore(&c->spinlock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
    unsigned long flags;
	unsigned long rate;

	if(!clk->ops)
	{
	    return clk->rate;
	}
	
	if (!clk->ops->get_rate)
		return -EINVAL; 
		
	spin_lock_irqsave(&clk->spinlock, flags);
	rate = clk->ops->get_rate(clk);
    spin_unlock_irqrestore(&clk->spinlock, flags);

	return rate;
	
}
EXPORT_SYMBOL_GPL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
    unsigned long flags;
    int ret=0;

	if (!clk->ops || !clk->ops->set_rate)
		return -EINVAL;
	if((clk->rate == rate) && (clk->ops != &leopard_clk_shared_bus_ops))
	{
	    return 0;
	}
	spin_lock_irqsave(&clk->spinlock, flags);
	ret = clk->ops->set_rate(clk, rate);
//	if (ret == 0)
//	{
//	    printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);
//		propagate_rate(clk);
//		printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);
//	}
	spin_unlock_irqrestore(&clk->spinlock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk->ops && clk->ops->round_rate)
		return clk->ops->round_rate(clk, rate);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(clk_round_rate);

void clk_reparent(struct clk *child, struct clk *parent)
{
	list_del_init(&child->sibling);
	if (parent)
    {
		list_add(&child->sibling, &parent->children);
    }
	child->parent = parent;

}

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret;
    unsigned long flags;
	if (!clk->ops || !clk->ops->set_parent)
		return -EINVAL;

    if(clk->parent == parent)
    {
        return 0;
    }

    spin_lock_irqsave(&clk->spinlock, flags);
	ret = clk->ops->set_parent(clk, parent);
	if (ret == 0)
	{
	    clk_reparent(clk, parent);
		propagate_rate(clk);
	}
	spin_unlock_irqrestore(&clk->spinlock, flags);
    //asoc_clock_debugfs_update_parent(clk);
	return ret;
}
EXPORT_SYMBOL_GPL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
    if(!clk->ops)
    {
        return clk->parent;
    }
	if (!clk->ops->get_parent)
		return NULL;
	return clk->ops->get_parent(clk);
}
EXPORT_SYMBOL_GPL(clk_get_parent);


struct clk *clk_get_by_name(const char *name)
{
	struct clk *clk=NULL;

	list_for_each_entry(clk, &asoc_clocks, list)
    {
		if (strcmp(clk->name, name) == 0)
			break;
        if (clk->alias != NULL && strcmp(clk->alias, name) == 0)
            break;
	}
	return clk;
}
EXPORT_SYMBOL_GPL(clk_get_by_name);

struct clk *clk_get_by_id(unsigned int id)
{
	struct clk *clk;

	list_for_each_entry(clk, &asoc_clocks, list)
    {
		if (clk->module_id == id)
			return clk;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(clk_get_by_id);


/* Propagate rate to children */
void propagate_rate(struct clk *tclk)
{
	struct clk *clkp;
        
    if((tclk == NULL) || (list_empty(&(tclk->children))))
    {
        return;
    }

	list_for_each_entry(clkp, &tclk->children, sibling) {
		if (clkp && clkp->ops && clkp->ops->recalc)
		{
			clkp->rate = clkp->ops->recalc(clkp);
		}
		propagate_rate(clkp);
	}
}

/**
 * recalculate_root_clocks - recalculate and propagate all root clocks
 *
 * Recalculates all root clocks (clocks with no parent), which if the
 * clock's .recalc is set correctly, should also propagate their rates.
 * Called at init.
 */
void recalculate_root_clocks(void)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &root_clks, sibling) {
        if(!clkp)
        {
            printk("\n error at %s %d", __FUNCTION__, __LINE__);
            return;
        }
		if (clkp->ops && clkp->ops->recalc)
		{
			clkp->rate = clkp->ops->recalc(clkp);
		}
		propagate_rate(clkp);
	}
}

int clk_is_enabled(struct clk *clk)
{
	if (clk->ops && clk->ops->is_enabled)
		return clk->ops->is_enabled(clk);

	return 1;
}
EXPORT_SYMBOL_GPL(clk_is_enabled);