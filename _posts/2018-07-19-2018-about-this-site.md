---
layout: post
title: "About This Site"
author: "Nick Rotella"
categories: journal
tags: [documentation,sample]
image: html.jpg
---

I've already fallen waaaaay behind what I was planning in terms of publishing posts on the python robot simulator I was working on, so instead I'm going to make a much easier post so that I actually get something out there.  It feels like half the battle of having a blog is getting in the habit of posting regularly, especially on days like today when scrambling to finish a huge post doesn't sound too appealing.

So, for a the past five years, I've actively been avoiding making a personal webpage because I was able to use [my lab's page](http://www-clmc.usc.edu/Main/NickRotella){:target="blank_"} and because, even though I'd taken a pretty comprehensive (if only surface-level) [course in web technologies](http://cs-server.usc.edu:45678/courseinfo.html){:target="_blank"}, I found the task to be daunting.  When I finally decided to *bite the bullet* (apparently this phrase dates from an 1891 novel, never stopped to think about it before) I was faced with the choice of either using my seriously-lapsed web development skills to build something from scratch, or stick with a nice WordPress theme - or so I thought.

It turns out that Github provides free hosting for one static site or [Github Page](https://help.github.com/articles/what-is-github-pages/){:target="_blank"} per account!  This seemed like a great middle ground because there are plenty of great premade "themes" out there, but in the end this means forking a repo of HTML/CSS/Javascript code which you can customize endlessly while benfitting from version control.  Plus, it integrates nicely with my Github profile - which I hope to flesh out with past and future projects - and fits in with my command line-based workflow.

Github Pages is also nice because you can draft new web pages by making local pages and using [Jekyll](https://jekyllrb.com/){:target="blank_"}, a Ruby-based static site generator.  After [setup](https://help.github.com/articles/setting-up-your-github-pages-site-locally-with-jekyll/){:target="blank_"}, simply run ```bundle exec jekyll serve``` from the root of your webpage repo and navigate to ```http://localhost:4000/``` in a browser to view the changes you've made locally! You can even connect to the Jekyll-generated site from a mobile device if you run ```bundle exec jekyll serve --host=0.0.0.0``` instead, allowing you to develop mobile-friendly webpages.

Speaking of mobile-friendly, you may have noticed that this site is barely so. Instead of sticking with a respondiv etheme based on [Bootstrap](https://getbootstrap.com/javascript/){:target="blank_"}, I chose to start from the minimalist [Lagrange theme](https://github.com/LeNPaul/Lagrange){:target="blank_"} and try ton refresh my limited web dev knowledge by manually in more dynamic features (like the morphing top menu).  I liked this theme in particular because it was blog-focused and felt less cluttered than some more "modern" themes.  It was also easy to integrate Disqus for commenting.  Speaking of blog posts, I'm writing these in [Markdown](){:target="blank_} which is a nifty little markup language which makes it easy to write without worrying about actually using HTML.  It also has $$\LaTeX$$ support via [MathJAX](https://www.mathjax.org/){:target="blank_"} which is essential for an academic/technical site (in fact, the Stack* sites use this for embedding math). 

$$\beta = \sum_{i}\begin{bmatrix}\alpha_{i}^{2}\\ \frac{\gamma_{i}^{3}}{2}\end{bmatrix}$$

You can do a heck of a lot on your own with HTML, CSS and Javascript, but if you really only care about easily building a mobile-first site then Bootstrap is probably (definitely) the way to go since all the tricky work is done.  Maybe I'll switch themes at some point, but with some minor tweaks I got this site to at least not completely break on mobile (at least on my Android device).  And it was a fun process, too! 
